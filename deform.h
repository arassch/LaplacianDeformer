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

#include <QGLViewer/qglviewer.h>
#include <TriMesh.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>


void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove);
void removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove);
TriEdge* getEdgeInFace(TriFace* f, TriVertex* v1, TriVertex* v2);

class Deformer
{
    TriMesh* m_mesh;

    Eigen::SparseMatrix<float> m_Ls;
    Eigen::VectorXf m_M;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float> > m_solver;

    double m_Ks, m_Kb;

public:
    Deformer(TriMesh* mesh);

    void prepareToMove(std::vector<int> &fixedVertices);
    void deform(std::vector<int> &fixedVertices, std::vector<Vector3> &positions);

    void setKs(double Ks);
    void setKb(double Kb);
};

class DeformViewer : public QGLViewer {
    Q_OBJECT
private:


    TriMesh* m_mesh;
    Deformer* m_deformer;

    enum SelectionMode { NONE, ADD, REMOVE };
    SelectionMode m_selectionMode;

    // Current rectangular selection
    QRect m_rectangle;

    QList<int> m_selection;
    QList<int> m_fixed;

    bool m_move;
    QPoint m_initialPoint;

public:
    DeformViewer(QWidget *parent);

public Q_SLOTS:
    void KsValueChanged(double value)
    {
        if(m_deformer)
            m_deformer->setKs(value);
    }

    void KbValueChanged(double value)
    {
        if(m_deformer)
            m_deformer->setKb(value);
    }

protected:
    virtual void draw();
    virtual void init();
    virtual QString helpString() const;

    void keyPressEvent(QKeyEvent *key);

    // Mouse events functions
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);

    void drawSelectionRectangle() const;
    void addIdToSelection(int id);
    void removeIdFromSelection(int id);



};


