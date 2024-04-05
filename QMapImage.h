#ifndef QMAPIMAGE_H
#define QMAPIMAGE_H

#include <QWidget>

namespace Ui {
class QMapImage;
}

#include "stdafx.h"
#include "amGL/CImage.h"
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include "3d/dhVector.h"

class QMapImage : public QWidget
{
    Q_OBJECT

public:
    explicit QMapImage(QWidget *parent = nullptr);
    ~QMapImage();

public:
    CImage  img;
    struct _info
    {
        dhVector lo;
        QImage lmap;

        dhVector o;        
        QImage map;

        int x,y,z;
        int xo,yo,zo;
        int xs,ys,zs;
        float zoom,zoomo;
        int w,h;
        bool bLock;
        float mps;  // meter per pixel
    } info;

    void    paintEvent(QPaintEvent*);
    void    UpdateMap(unsigned char *p,int n);
    void    SaveMap(BYTE *,int,int,char *psz);
    void    Lock()  {   info.bLock  = true; }
    void    Unlock(){   info.bLock  = false; }
    void    P2M(float x,float y, float& mx,float& my);
    void    M2P(float mx,float my, float& x,float& y);
signals:
    void    onTarget(float,float );

private:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *ev);
    void OnMouseMove(int nFlags, int x,int y);
    Ui::QMapImage *ui;
};

#endif // QMAPIMAGE_H
