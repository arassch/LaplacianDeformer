/****************************************************************************

 Copyright (C) 2002-2014 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.7.0.

 http://www.libqglviewer.com - contact@libqglviewer.com

 This file may be used under the terms of the GNU General Public License
 versions 2.0 or 3.0 as published by the Free Software Foundation and
 appearing in the LICENSE file included in the packaging of this file.
 In addition, as a special exception, Gilles Debunne gives you certain
 additional rights, described in the file GPL_EXCEPTION in this package.

 libQGLViewer uses dual licensing. Commercial/proprietary software must
 purchase a libQGLViewer Commercial License.

 This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#include "deform.h"
#include <math.h>
#include <TriVertex.h>
#include <set>
#include <Eigen/SparseCholesky>

void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}


// Constructor must call the base class constructor.
DeformViewer::DeformViewer(QWidget *parent) : QGLViewer(parent) {

    m_mesh = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/plane.obj", TriMesh::OBJ);
//    m_mesh = m_mesh->GetSubdividedMeshLinked(2);
    m_mesh->GetFaceProperties()->edges = true;
    m_mesh->GetFaceProperties()->invertNormal = true;
    Vector3 blue(1.0f, 1.0f, 0.8f);
    m_mesh->setColor(blue);

    m_deformer = new Deformer(m_mesh);

    m_selectionMode = NONE;
    m_move = false;
}

void DeformViewer::init() {
  // Used to display semi-transparent relection rectangle
  glBlendFunc(GL_ONE, GL_ONE);

  restoreStateFromFile();
  help();
}

void DeformViewer::keyPressEvent(QKeyEvent *key)
{
    if(key->text() == "w")
    {
        m_move = !m_move;
        if(m_move)
        {
            std::vector<int> constraintVerts;
            std::vector<TriVertex*> verts = m_mesh->Vertices();
            QList<int>::iterator it = m_selection.begin();
            for(;it!=m_selection.end();++it)
            {
                TriVertex* v = verts[*it];
                constraintVerts.push_back(v->Id());
            }
//            m_deformer->prepareToMove(constraintVerts);
        }
    }
    else if(key->text() == "s")
    {
        m_fixed = m_selection;
        m_selection.clear();
        update();
    }
    else if(key->text() == "q")
    {
        std::vector<int> constraintVerts;
        std::vector<Vector3> displacements;
        std::vector<TriVertex*> verts = m_mesh->Vertices();
        auto it = m_selection.begin();
        for(;it!=m_selection.end();++it)
        {
            TriVertex* v = verts[*it];

            constraintVerts.push_back(v->Id());
            displacements.push_back(Vector3(0,0.1,0));

            v->SetCoords0(v->PositionInit()+Vector3(0,0.1,0));
        }

        m_deformer->deform(constraintVerts, displacements);

        update();
    }
    else
        QGLViewer::keyPressEvent(key);
}


//   C u s t o m i z e d   m o u s e   e v e n t s

void DeformViewer::mousePressEvent(QMouseEvent *e) {
    // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
    m_rectangle = QRect(e->pos(), e->pos());

    if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::ShiftModifier))
        m_selectionMode = ADD;
    else if ((e->button() == Qt::LeftButton) &&
             (e->modifiers() == Qt::ControlModifier))
        m_selectionMode = REMOVE;
    else if(m_move)
    {
        m_initialPoint = e->pos();
    }
    else {
        //    if (e->modifiers() == Qt::ControlModifier)
        //      startManipulation();

        QGLViewer::mousePressEvent(e);
    }
}

void DeformViewer::mouseMoveEvent(QMouseEvent *e) {
    if (m_selectionMode != NONE) {
        // Updates m_rectangle coordinates and redraws rectangle
        m_rectangle.setBottomRight(e->pos());
        update();
    }
    else if(m_move)
    {
        QPoint currentPos = e->pos();
        QPoint displacement = (currentPos - m_initialPoint);
        qglviewer::Vec up = camera()->upVector();
        qglviewer::Vec right = camera()->rightVector();
        int width = camera()->screenWidth();
        int height = camera()->screenHeight();
        float realX = (float) displacement.x() / width;
        float realY = (float) displacement.y() / height;
        qglviewer::Vec v = up*realY*-1.0 + right*realX;

        Vector3 u(v[0], v[1], v[2]);
        QList<int>::iterator it = m_selection.begin();
        std::vector<TriVertex*> verts = m_mesh->Vertices();

        std::vector<int> constraintVerts;
        std::vector<Vector3> displacements;
        for(;it!=m_selection.end();++it)
        {
            TriVertex* v = verts[*it];
            Vector3 pos = v->PositionInit();

            qglviewer::Vec posSP = camera()->projectedCoordinatesOf(qglviewer::Vec(pos));
            posSP.x += displacement.x();
            posSP.y += displacement.y();

            qglviewer::Vec disp = camera()->unprojectedCoordinatesOf(posSP);
            Vector3 p(disp.x, disp.y, disp.z) ;

            v->Position(p[0],p[1],p[2]);

            constraintVerts.push_back(v->Id());
            displacements.push_back(p-v->PositionInit());
        }
        it = m_fixed.begin();
        for(;it!=m_fixed.end();++it)
        {
            TriVertex* v = verts[*it];
            constraintVerts.push_back(v->Id());
            displacements.push_back(Vector3(0,0,0));
        }
        m_deformer->deform(constraintVerts, displacements);
        update();

    }
    else
    {

        QGLViewer::mouseMoveEvent(e);
    }
}

void DeformViewer::mouseReleaseEvent(QMouseEvent *e) {
    if (m_selectionMode != NONE) {
        bool somethingSelected = false;

        // Actual selection on the rectangular area.
        std::vector<TriVertex*> verts = m_mesh->Vertices();
        for(unsigned i=0;i<verts.size();++i)
        {
            TriVertex* v = verts[i];
            Vector3 p = v->Position();
            qglviewer::Vec src;
            src[0] = p[0];
            src[1] = p[1];
            src[2] = p[2];

            qglviewer::Vec w = camera()->projectedCoordinatesOf(src);
            if (m_rectangle.contains(w.x,w.y))
            {
                if(m_selectionMode == ADD)
                    addIdToSelection(i);
                else if (m_selectionMode == REMOVE)
                    removeIdFromSelection(i);
                somethingSelected = true;
            }
        }
        glEnd();

        if(!somethingSelected)
            m_selection.clear();

        m_selectionMode = NONE;
        // Update display to show new selected objects
        update();
    }
    else if(m_move)
    {
        std::vector<TriVertex*> verts = m_mesh->Vertices();
        for(unsigned i=0;i<verts.size();++i)
        {
            TriVertex* v = verts[i];
            v->SetCoords0(v->Position());
        }
    } else
        QGLViewer::mouseReleaseEvent(e);
}

void DeformViewer::draw() {
    m_mesh->Draw();

    glDisable(GL_LIGHTING);
    glPointSize(10);
    std::vector<TriVertex*> verts = m_mesh->Vertices();

    glColor3f(1.0f,0.0,0.0f);
    glBegin(GL_POINTS);
    QList<int>::iterator it = m_selection.begin();
    for(;it!=m_selection.end();++it)
    {
        if(*it < 0 || *it >= (int) verts.size())
            continue;
        TriVertex* v = verts[*it];
        Vector3 p = v->Position();
        glVertex3d(p[0],p[1],p[2]);
    }
    glEnd();

    glColor3f(0.7,0.7,0.7f);
    glBegin(GL_POINTS);
    it = m_fixed.begin();
    for(;it!=m_fixed.end();++it)
    {
        if(*it < 0 || *it >= (int) verts.size())
            continue;
        TriVertex* v = verts[*it];
        Vector3 p = v->Position();
        glVertex3d(p[0],p[1],p[2]);
    }
    glEnd();

    glEnable(GL_LIGHTING);

    // Draws rectangular selection area. Could be done in postDraw() instead.
    if (m_selectionMode != NONE)
        drawSelectionRectangle();
}

//   S e l e c t i o n   t o o l s

void DeformViewer::addIdToSelection(int id) {
    if (!m_selection.contains(id))
    {
        m_selection.push_back(id);
    }
}

void DeformViewer::removeIdFromSelection(int id) { m_selection.removeAll(id); }

//void DeformViewer::KsValueChanged(double value)
//{
//    cout << value << endl;
//}

void DeformViewer::drawSelectionRectangle() const {
    startScreenCoordinatesSystem();
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);

    glColor4f(0.0, 0.0, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2i(m_rectangle.left(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.bottom());
    glVertex2i(m_rectangle.left(), m_rectangle.bottom());
    glEnd();

    glLineWidth(2.0);
    glColor4f(0.4f, 0.4f, 0.5f, 0.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(m_rectangle.left(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.bottom());
    glVertex2i(m_rectangle.left(), m_rectangle.bottom());
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    stopScreenCoordinatesSystem();
}


QString DeformViewer::helpString() const {
    QString text("<h2>I n t e r f a c e</h2>");
    text += "A GUI can be added to a QGLViewer widget using Qt's "
            "<i>Designer</i>. Signals and slots ";
    text += "can then be connected to and from the viewer.<br><br>";
    text += "You can install the QGLViewer designer plugin to make the QGLViewer "
            "appear as a ";
    text += "standard Qt widget in the Designer's widget tabs. See installation "
            "pages for details.";
    return text;
}

void Deformer::setKs(double Ks)
{
    m_Ks = Ks;
}

void Deformer::setKb(double Kb)
{
    m_Kb = Kb;
}

Deformer::Deformer(TriMesh *mesh)
{
    m_mesh = mesh;
    m_Ks = 1.0;
    m_Kb = 1.0;

    std::vector<Eigen::Triplet<float> > data;
    std::vector<TriVertex*> verts = mesh->Vertices();
    std::vector<TriFace*> faces = mesh->Faces();
    for(unsigned i=0; i<verts.size();++i)
    {
        TriVertex* v = verts[i];
        std::vector<int>* incidentFaces = v->GetIncidentFaces();
        std::set<int> incidentFacesSet;

        std::set<TriVertex*> neighbors;
        for(unsigned j=0;j<incidentFaces->size();++j)
        {
            TriFace* f = faces[(*incidentFaces)[j]];
            if(f->Vertex1() != v)
                neighbors.insert(f->Vertex1());
            if(f->Vertex2() != v)
                neighbors.insert(f->Vertex2());
            if(f->Vertex3() != v)
                neighbors.insert(f->Vertex3());

            incidentFacesSet.insert(f->Id());
        }

        LinalFloat sum=0;
        std::set<TriVertex*>::iterator it = neighbors.begin();
        for(int j=0;it!=neighbors.end();++it, ++j)
        {
            TriEdge* e = v->AdjEdge(*it);
            if(e == NULL)
                e = (*it)->AdjEdge(v);

            LinalFloat a = e->Length();
            LinalFloat b = e->Next()->Length();
            LinalFloat c = e->Prev()->Length();

            LinalFloat alpha = (b*b + c*c - a*a) / (2*b*c);
            LinalFloat cotAlpha = 1.0/tan(alpha);
            if(tan(alpha) < 0.001)
                cotAlpha = 0;

            e = e->Twin();
            LinalFloat wij;
            if(e)
            {
                b = e->Next()->Length();
                c = e->Prev()->Length();
                LinalFloat beta = (b*b + c*c - a*a) / (2*b*c);
                LinalFloat cotBeta = 1.0/tan(beta);
                if(tan(beta) < 0.001)
                    cotBeta = 0;
                wij = 0.5*(cotAlpha + cotBeta);
            }
            else
                wij = 0.5*cotAlpha;


            sum += wij;
            data.push_back(Eigen::Triplet<float>(v->Id(), (*it)->Id(), wij));
        }
        data.push_back(Eigen::Triplet<float>(v->Id(), v->Id(), -1.0f*sum));

    }

    m_Ls.resize(verts.size(), verts.size());
    m_Ls.setFromTriplets(data.begin(), data.end());

    Eigen::VectorXf diag(verts.size());
    for(unsigned i=0; i<verts.size();++i)
    {
        TriVertex* v = verts[i];
        std::vector<int>* incidentFaces = v->GetIncidentFaces();

        LinalFloat area = 0.0;
        for(unsigned j=0;j<incidentFaces->size();++j)
        {
            TriFace* f = faces[(*incidentFaces)[j]];

            TriVertex *v1=0, *v2=0;
            if(f->Vertex1() != v)
                    v1 = f->Vertex1();
            if(f->Vertex2() != v)
            {
                if(!v1)
                    v1 = f->Vertex2();
                else
                    v2 = f->Vertex2();
            }
            if(f->Vertex3() != v)
                v2 = f->Vertex3();

            TriEdge* e1 = v->AdjEdge(v1);
            if(e1 == NULL)
                e1 = v1->AdjEdge(v);
            TriEdge* e2 = v->AdjEdge(v2);
            if(e2 == NULL)
                e2 = v2->AdjEdge(v);
            TriEdge* e3 = v1->AdjEdge(v2);
            if(e3 == NULL)
                e3 = v2->AdjEdge(v1);
            LinalFloat l1 = e1->Length();
            LinalFloat l2 = e2->Length();
            LinalFloat l3 = e3->Length();
            LinalFloat angle = (l1*l1+l2*l2-l3*l3)/(2*l1*l2);

            Vector3 centroid;
            if(angle > M_PI_2)
            {
                centroid = 0.5*(v1->Position()+v2->Position());
            }
            else
            {
                centroid = f->getCentroid();
            }

            Vector3 centE1 = 0.5*(v->Position()+v1->Position());
            Vector3 centE2 = 0.5*(v->Position()+v2->Position());

            Vector3 vec = Vector3::crossProd((centE1 - centroid),(v->Position() - centroid));
            area += 0.5f * vec.getLength();

            vec = Vector3::crossProd((centE2 - centroid),(v->Position() - centroid));
            area += 0.5f * vec.getLength();
        }
        diag[v->Id()] = area;
    }

    m_M = diag;
}

void Deformer::prepareToMove(std::vector<int> &constraintVertices)
{
    float ks=100, kb=1;

    Eigen::MatrixXf A = -ks*m_Ls + kb*m_Ls*m_M.asDiagonal().inverse()*m_Ls;

    std::set<int> setConstraints;

    for(unsigned i=0;i<constraintVertices.size();++i)
    {
        int rowToRemove = constraintVertices[i]-i;
        int numRows = A.rows()-1;
        int numCols = A.cols();
        if( rowToRemove < numRows )
        {
            A.block(rowToRemove,0,numRows-rowToRemove,numCols) = A.block(rowToRemove+1,0,numRows-rowToRemove,numCols);
        }

        A.conservativeResize(numRows,numCols);

        setConstraints.insert(constraintVertices[i]);
    }

    for(unsigned i=0;i<constraintVertices.size();++i)
    {
        int colToRemove = constraintVertices[i]-i;

        int numRows = A.rows();
        int numCols = A.cols()-1;
        if( colToRemove < numCols )
            A.block(0,colToRemove,numRows,numCols-colToRemove) = A.block(0,colToRemove+1,numRows,numCols-colToRemove);

        A.conservativeResize(numRows,numCols);
    }

    Eigen::SparseMatrix<float> Asparse = A.sparseView();
    m_solver.compute(Asparse);

}

void Deformer::deform(std::vector<int> &constraintVertices, std::vector<Vector3> &displacements)
{
    std::map<int, Vector3> mapConstraint;
    for(unsigned i=0;i<constraintVertices.size();++i)
        mapConstraint[constraintVertices[i]] = displacements[i];

    Eigen::MatrixXf A = -m_Ks*m_Ls + m_Kb*m_Ls*m_M.asDiagonal().inverse()*m_Ls;
    Eigen::MatrixXf L = m_M.asDiagonal().inverse()*m_Ls;
    Eigen::MatrixXf C = -m_Ks*L + m_Kb*L*L, Csmall;
    Eigen::VectorXf vM;

    vector<Eigen::Triplet<float> > data;

    Csmall.resize(A.rows()-mapConstraint.size(), A.cols());
    vM.resize(A.rows()-mapConstraint.size());
    for(int r=0,ri=0; r<A.rows(); ++r)
    {
        if(mapConstraint.find(r) != mapConstraint.end())
            continue;

        vM[ri] = m_M[r];
        for(int c=0,ci=0; c<A.cols(); ++c)
        {
            Csmall(ri,c) = C(r,c);

            if(mapConstraint.find(c) != mapConstraint.end())
                continue;
            if(fabs(A(r,c)) > 0.0001)
                data.push_back(Eigen::Triplet<float>(ri,ci,A(r,c)));

            ci++;
        }

        ri++;
    }
    Eigen::SparseMatrix<float> Asparse(A.rows()-mapConstraint.size(), A.rows()-mapConstraint.size());
    Asparse.setFromTriplets(data.begin(), data.end());


    Eigen::VectorXf bx(Csmall.rows()), by(Csmall.rows()), bz(Csmall.rows());
    bx.setZero(Csmall.rows());
    by.setZero(Csmall.rows());
    bz.setZero(Csmall.rows());

    m_solver.compute(Asparse);

    auto it = mapConstraint.begin();
    for(unsigned i=0;it!=mapConstraint.end();++i,++it)
    {
        int colToRemove = it->first;
        bx -= Csmall.col(colToRemove) * it->second[0];
        by -= Csmall.col(colToRemove) * it->second[1];
        bz -= Csmall.col(colToRemove) * it->second[2];
    }

    Eigen::DiagonalMatrix<float, Eigen::Dynamic> M = vM.asDiagonal();
    Eigen::VectorXf dx = m_solver.solve(M*bx);
    Eigen::VectorXf dy = m_solver.solve(M*by);
    Eigen::VectorXf dz = m_solver.solve(M*bz);

    std::vector<TriVertex*> verts = m_mesh->Vertices();
    for(unsigned i=0, j=0;i<verts.size();++i)
    {
        if(mapConstraint.find(i) != mapConstraint.end())
            continue;

        Vector3 p = verts[i]->PositionInit() + Vector3(dx[j],dy[j],dz[j]);
        verts[i]->Position(p[0],p[1],p[2]);

        j++;
    }
}

