#include "QMapImage.h"
#include "ui_QMapImage.h"
#include <QDebug>
#include "3d/dhVector.h"

QMapImage::QMapImage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QMapImage)
{
    info.zoom   = 5;
    info.x      = (-2000)*5+400;
    info.y      = (-2000)*5+400;
    info.xs     = 0;
    info.ys     = 0;
    info.z      = 255;
    info.w      = 4000;
    info.h      = 4000;
    info.bLock  = false;
    info.mps    = 0.05;

    ui->setupUi(this);

    // loading map
    if (img.Load("map/map_bokk.pgm"))
    {
        info.map= QImage(img.w,img.h,QImage::Format_Grayscale8);
        memmove(info.map.bits(),img.GetBuffer(),img.w*img.h);
    }
    else
        info.map= QImage(info.w,info.h,QImage::Format_Grayscale8);

}

QMapImage::~QMapImage()
{
    delete ui;
}

void QMapImage::paintEvent(QPaintEvent *event)
{
    QPainter dc(this);    

    dc.scale(info.zoom,info.zoom);
    dc.drawImage(info.x/info.zoom,info.y/info.zoom,info.map);

    if (1) //local map
    {
        float x,y;
        M2P(info.lo.x,-info.lo.y,x,y);

        info.lmap.transformed(QMatrix().rotate(info.o.z));
        y-=info.lmap.height();
        dc.drawImage(x,y,info.lmap);
    }

    // robot
    float ox= info.o.x;
    float oy= info.o.y;
    float q = info.o.z; // deg
    float x,y,x2,y2,x3,y3;
    float px,py,px1,py1,px2,py2,px3,py3;

    x   = ox+1*cos(RAD(q));
    y   = oy+1*sin(RAD(q));
    x2  = ox+0.5*cos(RAD(q+120));
    y2  = oy+0.5*sin(RAD(q+120));
    x3  = ox+0.5*cos(RAD(q-120));
    y3  = oy+0.5*sin(RAD(q-120));

    // meter to pixel
    M2P(ox,oy,px,py);
    M2P(x,y,px1,py1);
    M2P(x2,y2,px2,py2);
    M2P(x3,y3,px3,py3);

    QPolygonF points({{px1,py1},{px2,py2},{px3,py3},{px1,py1}});
    dc.setPen(QPen(Qt::red, 1.0f));
    dc.drawPolygon(points);
    dc.drawLine(QPointF(px,py),QPointF(px1,py1));
}

void QMapImage::P2M(float x,float y,float& mx,float& my)
{
    float tx,ty;
    tx  = (-info.x)/info.zoom + x/info.zoom;
    ty  = (-info.y)/info.zoom + y/info.zoom;

    mx  = tx-info.w/2;
    my  = ty-info.h/2;
    mx  = mx*info.mps;
    my  = my*info.mps;
}

void QMapImage::M2P(float mx,float my,float& x,float& y)
{
    mx/=info.mps;
    my/=info.mps;

    float tx    = mx+info.w/2;
    float ty    = my+info.h/2;

    x   =tx*info.zoom+info.x;
    y   =ty*info.zoom+info.y;

    x/=info.zoom;
    y/=info.zoom;
}

void QMapImage::mousePressEvent(QMouseEvent *event)
{
    QPoint pt = event->pos();

    int n = event->buttons();
    int x = pt.x();
    int y = pt.y();
    int tx,ty;
    float fx,fy;

    switch(n){
    case Qt::LeftButton :
        P2M(x,y,fx,fy);

        info.xs     = x;        info.ys =   y;
        info.xo     = info.x;   info.yo =   info.y;

        if (info.bLock)
        emit onTarget(fx,-fy);
        break;

    case Qt::RightButton:
        info.xs     = x;        info.ys =   y;
        info.xo     = info.x;   info.yo =   info.y;
        info.zs     = y;
        info.zo     = info.z;
        info.zoomo  = info.zoom;
        break;
    }    
}

void QMapImage::wheelEvent(QWheelEvent *ev)
{
    if (info.bLock) return;

    int dz   = ev->angleDelta().y();

    int x   = ev->pos().x();
    int y   = ev->pos().y();

    info.xs     = x;        info.ys =   y;
    info.xo     = info.x;   info.yo =   info.y;
    info.zs     = y;
    info.zo     = info.z;
    info.zoomo  = info.zoom;

    info.z  = info.zo + dz*0.1;
    if (info.z<0)   info.z;
    if (info.z>256) info.z = 256;

    float f;
    f   = 0.2+ ((float)info.z)/50.;
    f   = MAX(f,0.2);
    f   = MIN(f,5);

    info.x  = (info.xo-info.xs)*f/info.zoomo+ info.xs;
    info.y  = (info.yo-info.ys)*f/info.zoomo+ info.ys;
    info.zoom   = f;
    update();
}

void QMapImage::mouseMoveEvent(QMouseEvent *event)
{
    QPoint pt = event->pos();

    int n = event->buttons();

    if (!info.bLock)
    switch(n){
    case Qt::LeftButton :   OnMouseMove(MK_LBUTTON, pt.x(),pt.y());   break;
    case Qt::RightButton:   OnMouseMove(MK_RBUTTON, pt.x(),pt.y());   break;
    }
}

void QMapImage::OnMouseMove(int nFlags, int x, int y)
{
    float f;
    int dx,dy,dz;
    switch(nFlags){
    case MK_LBUTTON:
        dx  = x-info.xs;
        dy  = y-info.ys;
        info.x  = info.xo+dx;
        info.y  = info.yo+dy;
        update();
        break;
    case MK_RBUTTON:
        dz  = info.zs-y;
        info.z  = info.zo + dz;
        if (info.z<0)   info.z;

        f   = 0.2+ ((float)info.z)/50.;
        f   = MAX(f,0.2);
        f   = MIN(f,5);

        info.x  = (info.xo-info.xs)*f/info.zoomo+ info.xs;
        info.y  = (info.yo-info.ys)*f/info.zoomo+ info.ys;
        info.zoom   = f;
        update();
        break;
    }
}

void QMapImage::UpdateMap(BYTE *pSrc,int n)
{
    BYTE *pDst  = info.map.bits();
    int i;
    memmove(pDst,pSrc,n);

    // rearrange
    for (i=0;i<n;i++)
    {
        if (pSrc[i]==255)   continue;
        if (pSrc[i]==100)   pDst[i] = 0;
        else                pDst[i] = 160;
    }

    // flip vertically
    memmove(pSrc,pDst,n);
    BYTE *ps,*pt;
    ps  = pSrc;
    pt  = &pDst[info.w*(info.h-1)];

    for (i=0;i<info.h;i++)
    {
        memmove(pt,ps,info.w);
        pt-=info.w;
        ps+=info.w;
    }

    update();
}

void QMapImage::SaveMap(BYTE *p,int w,int h,char *psz)
{
    vString tmp;

    float res   = 0.05;
    float ox    = -100;
    float oy    = -100;
    float oq    = 0;
    int negate  = 0;
    float occth = 0.65;
    float freeth= 0.196;

    // PGM
    CImage img;
    img.bits    = 8;
    img.ResetBuffer(w,h);
    memmove(img.GetBuffer(), p,w*h);
    tmp.Format("%s.pgm",psz);
    img.PGMWrite((char*)tmp);

    // YAML
    tmp.Format("%s.yaml",psz);
    FILE *fp    = fopen((char*)tmp,"wb");
    tmp.Format("image: %s.pgm\n",psz);
    fwrite((char*)tmp,tmp.GetLength(),1,fp);

    tmp.Format("resolution: %f\n", res);
    fwrite((char*)tmp,tmp.GetLength(),1,fp);

    tmp.Format("origin: [ %f, %f, %f]\n",ox,oy,oq );
    fwrite((char*)tmp,tmp.GetLength(),1,fp);

    tmp.Format("negate: %d\n", negate);
    fwrite((char*)tmp,tmp.GetLength(),1,fp);

    tmp.Format("occupied_thresh: %f\n", occth);
    fwrite((char*)tmp,tmp.GetLength(),1,fp);

    tmp.Format("free_thresh: %f\n", freeth);
    fwrite((char*)tmp,tmp.GetLength(),1,fp);
    fclose(fp);

    // map to map
    memmove(info.map.bits(),info.map.bits(),img.w*img.h);
}
