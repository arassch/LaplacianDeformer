// Thin C API so the browser viewer can drive Sara's Deformer.
// Mirrors what DeformViewer (deform.cpp) does in mouseMoveEvent / mouseReleaseEvent.
#include <emscripten/emscripten.h>
#include <vector>
#include <algorithm>
#include "deformer_extracted.cpp"

static void g_fastReset();
static TriMesh*  g_mesh = 0;
static Deformer* g_def  = 0;
static std::vector<float> g_pos;   // 3*N current positions
static std::vector<int>   g_tris;  // 3*F vertex ids

static void syncPositions()
{
    std::vector<TriVertex*> verts = g_mesh->Vertices();
    g_pos.resize(3 * verts.size());
    for (unsigned i = 0; i < verts.size(); ++i) {
        Vector3 p = verts[i]->Position();
        int id = verts[i]->Id();
        g_pos[3*id] = p[0]; g_pos[3*id+1] = p[1]; g_pos[3*id+2] = p[2];
    }
}

extern "C" {

// Load an OBJ from the in-memory filesystem with TriMesh::ReadFromFile. Returns vertex count (0 on failure).
EMSCRIPTEN_KEEPALIVE int ld_load(const char* path)
{
    delete g_def; g_def = 0;
    g_fastReset();
    g_mesh = TriMesh::ReadFromFile(path, TriMesh::OBJ);
    if (!g_mesh) return 0;
    std::vector<TriFace*> faces = g_mesh->Faces();
    g_tris.resize(3 * faces.size());
    for (unsigned f = 0; f < faces.size(); ++f) {
        g_tris[3*f]   = faces[f]->Vertex1()->Id();
        g_tris[3*f+1] = faces[f]->Vertex2()->Id();
        g_tris[3*f+2] = faces[f]->Vertex3()->Id();
    }
    g_def = new Deformer(g_mesh);
    syncPositions();
    return (int)g_mesh->Vertices().size();
}

EMSCRIPTEN_KEEPALIVE int    ld_numFaces()   { return (int)g_tris.size() / 3; }
EMSCRIPTEN_KEEPALIVE int*   ld_faces()      { return g_tris.data(); }
EMSCRIPTEN_KEEPALIVE float* ld_positions()  { return g_pos.data(); }

// Rest positions (PositionInit) — the drag is measured from these, as in DeformViewer.
EMSCRIPTEN_KEEPALIVE float ld_restCoord(int id, int axis)
{
    return (float)g_mesh->Vertices()[id]->PositionInit()[axis];
}

// All rest positions (PositionInit) at once, 3*N floats.
static std::vector<float> g_rest;
EMSCRIPTEN_KEEPALIVE float* ld_restPositions()
{
    std::vector<TriVertex*> verts = g_mesh->Vertices();
    g_rest.resize(3 * verts.size());
    for (unsigned i = 0; i < verts.size(); ++i) {
        Vector3 p = verts[i]->PositionInit(); int id = verts[i]->Id();
        g_rest[3*id] = p[0]; g_rest[3*id+1] = p[1]; g_rest[3*id+2] = p[2];
    }
    return g_rest.data();
}

EMSCRIPTEN_KEEPALIVE void ld_setKs(double v) { if (g_def) g_def->setKs(v); }
EMSCRIPTEN_KEEPALIVE void ld_setKb(double v) { if (g_def) g_def->setKb(v); }

} // extern "C"

// ---------------------------------------------------------------------------------------------
// Sparse, cached version of Deformer::deform. It solves the same system as Sara's deform():
//   A = -Ks*Ls + Kb*Ls*M^-1*Ls, restricted to the free vertices (entries |A| > 1e-4 kept, as in deform()),
//   right-hand side = -A(free, constrained) * d(constrained)   [deform() builds this as M * Csmall, which equals A's rows]
// but A is built sparse instead of dense, and the factorization is reused while the constraint set and Ks/Kb
// stay the same (i.e. for a whole drag), so each mouse move is only a back-substitution.
// ---------------------------------------------------------------------------------------------
struct FastSolve
{
    std::vector<int> key; double ks = -1, kb = -1; int n = 0;
    std::vector<int> freeIdx;                       // vertex id -> row in the free system, or -1
    Eigen::SparseMatrix<float> Aff, Afc;            // free x free (thresholded), free x all (constrained columns used)
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float> > solver;

    void prepare(Deformer& D, const std::vector<int>& cons)
    {
        std::vector<int> k(cons); std::sort(k.begin(), k.end());
        if (k == key && ks == D.m_Ks && kb == D.m_Kb && n == D.m_Ls.rows()) return;
        key = k; ks = D.m_Ks; kb = D.m_Kb; n = D.m_Ls.rows();

        Eigen::VectorXf minv = D.m_M.cwiseInverse();
        Eigen::SparseMatrix<float> LMinv = D.m_Ls * minv.asDiagonal();
        Eigen::SparseMatrix<float> A = (-(float)ks) * D.m_Ls + (float)kb * (LMinv * D.m_Ls);

        freeIdx.assign(n, 0);
        for (int c : key) freeIdx[c] = -1;
        int nfree = 0;
        for (int i = 0; i < n; ++i) if (freeIdx[i] != -1) freeIdx[i] = nfree++;

        std::vector<Eigen::Triplet<float> > tff, tfc;
        for (int c = 0; c < A.outerSize(); ++c)
            for (Eigen::SparseMatrix<float>::InnerIterator it(A, c); it; ++it) {
                int r = it.row(), fr = freeIdx[r];
                if (fr < 0) continue;
                int fc = freeIdx[c];
                if (fc >= 0) { if (fabs(it.value()) > 0.0001) tff.push_back(Eigen::Triplet<float>(fr, fc, it.value())); }
                else tfc.push_back(Eigen::Triplet<float>(fr, c, it.value()));
            }
        Aff.resize(nfree, nfree); Aff.setFromTriplets(tff.begin(), tff.end());
        Afc.resize(nfree, n);     Afc.setFromTriplets(tfc.begin(), tfc.end());
        solver.compute(Aff);
    }

    void solve(TriMesh* mesh, const std::vector<int>& cons, const std::vector<Vector3>& disp)
    {
        int nfree = Aff.rows();
        Eigen::VectorXf dc[3];
        for (int a = 0; a < 3; ++a) dc[a] = Eigen::VectorXf::Zero(n);
        for (unsigned i = 0; i < cons.size(); ++i) for (int a = 0; a < 3; ++a) dc[a][cons[i]] = disp[i][a];
        Eigen::VectorXf d[3];
        for (int a = 0; a < 3; ++a) { Eigen::VectorXf b = -(Afc * dc[a]); d[a] = solver.solve(b); }
        std::vector<TriVertex*> verts = mesh->Vertices();
        for (unsigned i = 0; i < verts.size(); ++i) {
            int fr = freeIdx[i];
            if (fr < 0) continue;
            Vector3 p = verts[i]->PositionInit() + Vector3(d[0][fr], d[1][fr], d[2][fr]);
            verts[i]->Position(p[0], p[1], p[2]);
        }
    }
};
static FastSolve g_fast;
static void g_fastReset() { g_fast.key.clear(); g_fast.n = -1; g_fast.ks = g_fast.kb = -1; }

static void gatherConstraints(const int* handles, const float* targets, int nh, const int* fixedIds, int nf,
                              std::vector<int>& cons, std::vector<Vector3>& disp)
{
    std::vector<TriVertex*> verts = g_mesh->Vertices();
    for (int i = 0; i < nh; ++i) {
        TriVertex* v = verts[handles[i]];
        Vector3 p(targets[3*i], targets[3*i+1], targets[3*i+2]);
        v->Position(p[0], p[1], p[2]);
        cons.push_back(v->Id());
        disp.push_back(p - v->PositionInit());
    }
    for (int i = 0; i < nf; ++i) {
        TriVertex* v = verts[fixedIds[i]];
        cons.push_back(v->Id());
        disp.push_back(Vector3(0, 0, 0));
    }
}

extern "C" {

// One mouse-move step: handles get new absolute positions (xyz per handle), fixed get zero displacement.
// Same constraints as the m_move branch of DeformViewer::mouseMoveEvent; solved with the sparse cached path.
EMSCRIPTEN_KEEPALIVE void ld_deform(const int* handles, const float* targets, int nh, const int* fixedIds, int nf)
{
    std::vector<int> cons; std::vector<Vector3> disp;
    gatherConstraints(handles, targets, nh, fixedIds, nf, cons, disp);
    g_fast.prepare(*g_def, cons);
    g_fast.solve(g_mesh, cons, disp);
    syncPositions();
}

// Sara's original Deformer::deform, unchanged (dense). Kept for comparison/testing.
EMSCRIPTEN_KEEPALIVE void ld_deformOriginal(const int* handles, const float* targets, int nh, const int* fixedIds, int nf)
{
    std::vector<int> cons; std::vector<Vector3> disp;
    gatherConstraints(handles, targets, nh, fixedIds, nf, cons, disp);
    g_def->deform(cons, disp);
    syncPositions();
}

// Mouse release in move mode: current shape becomes the new rest pose (SetCoords0), as in DeformViewer.
EMSCRIPTEN_KEEPALIVE void ld_commit()
{
    std::vector<TriVertex*> verts = g_mesh->Vertices();
    for (unsigned i = 0; i < verts.size(); ++i) verts[i]->SetCoords0(verts[i]->Position());
    syncPositions();
}

// Undo support: restore a saved shape as both current and rest pose (3*N floats).
EMSCRIPTEN_KEEPALIVE void ld_setShape(const float* p)
{
    std::vector<TriVertex*> verts = g_mesh->Vertices();
    for (unsigned i = 0; i < verts.size(); ++i) {
        int id = verts[i]->Id();
        verts[i]->SetCoords0(Vector3(p[3*id], p[3*id+1], p[3*id+2]));
    }
    syncPositions();
}
}
