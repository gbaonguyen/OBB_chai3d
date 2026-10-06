#include "CCollisionOBBNode.h"

namespace chai3d {

cCollisionOBBNode::cCollisionOBBNode()
{
    // Derived constructors replace this with the appropriate node kind.
    m_nodeType = C_COLLISION_OBB_NODE_INTERNAL;
}

cCollisionOBBNode::~cCollisionOBBNode() {}

} // namespace chai3d