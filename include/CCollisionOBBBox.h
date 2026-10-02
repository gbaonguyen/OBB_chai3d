#ifndef CCOLLISION_OBB_BOX_H
#define CCOLLISION_OBB_BOX_H

#include "math/CVector3d.h"

namespace chai3d{

class cCollisionOBBBox{
public:

    cVector3d m_center;
    cVector3d m_extent;

    cVector3d u[3];

    cCollisionOBBBox(){
        m_center.zero();
        m_extent.zero();

        u[0].set(1.0, 0.0, 0.0);
        u[1].set(0.0, 1.0, 0.0);
        u[2].set(0.0, 0.0, 1.0);
    }

    virtual ~cCollisionOBBBox() {}

};
    
}

#endif