#include "dynacore_yaml-cpp/node/node.h"
#include "nodebuilder.h"
#include "nodeevents.h"

namespace dynacore_YAML
{
Node Clone ( const Node& node )
{
    NodeEvents events ( node );
    NodeBuilder builder;
    events.Emit ( builder );
    return builder.Root();
}
}
