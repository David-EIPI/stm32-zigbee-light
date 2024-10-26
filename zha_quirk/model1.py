from zigpy.quirks import CustomCluster
from zigpy.quirks.v2 import QuirkBuilder
from zigpy.quirks.v2.homeassistant import EntityType
from zigpy.zcl.clusters.general import MultistateValue
import zigpy.types as t

# ZHA currently does not support StatesText attribute in Multistate clusters.
# We need to specify the states here.

class lightOperationModes(t.enum8):
    """Light operation modes."""

    Auto = 0x01
    Full = 0x02
    Dimmed = 0x03
    Off = 0x04


# ZHA incorrectly uses Float type for Present Value attribute in Multistate cluster. 
# According to ZCL spec this attribute type should be uint16.

class fixMultistateValue(CustomCluster, MultistateValue):
    attributes = MultistateValue.attributes.copy()
    attributes.update(
        {
            0x0055: ("present_value", t.uint16_t, True)
        }
    )

(
    QuirkBuilder("DS", "Model1")
    .enum(
        "present_value",
        lightOperationModes,
        MultistateValue.cluster_id,
        entity_type = EntityType.STANDARD,
        translation_key = "operation_mode",
        fallback_name = "Operation mode",
    )
    .replaces(fixMultistateValue)
    .add_to_registry()
)

