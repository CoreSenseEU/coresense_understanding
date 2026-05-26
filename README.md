## CoreSense Understanding System

### Purpose

This module generates strategies to generate models with given properties.
It uses existing models as well as model-modification skills (engines) wrapped in Behavior Trees and encapsulated in annotated ROS nodes.

### Installation

#### Manual
```bash
git clone --recursive https://github.com/CoreSenseEU/coresense_understanding src/coresense_understanding
git clone https://github.com/CoreSenseEU/coresense_msgs src/coresense_msgs
git clone https://github.com/CoreSenseEU/coresense_vampire src/coresense_vampire
git clone https://github.com/CoreSenseEU/triplestar_kb src/triplestar_kb
git clone https://github.com/CoreSenseEU/coresense_bringup src/coresense_bringup
git clone https://github.com/CoreSenseEU/coresense_understanding_bringup src/coresense_understanding_bringup
sudo apt install -y --no-install-recommends python3-pip ros-jazzy-tinyxml2-vendor ros-jazzy-nlohmann-json-schema-validator-vendor
# dependencies for triplestar_kb:
pip install --user --break-system-packages pydantic pyoxigraph reasonable oxrdflib shapely copier returns
colcon build --symlink-install
source install/setup.bash
```

#### Scripts
```bash
git clone https://github.com/CoreSenseEU/coresense_bringup src/coresense_bringup
git clone https://github.com/CoreSenseEU/coresense_understanding_bringup src/coresense_understanding_bringup
git clone --recursive https://github.com/CoreSenseEU/coresense_understanding src/coresense_understanding
cd coresense_understanding
./install.sh
cd ../triplestar_kb
./install.sh
cd ../..
colcon build --symlink-install
source install/setup.bash
```

#### Dependencies
- tinyxml2_vendor
- nlohmann_json_schema_validator_vendor
- [understanding-logic](https://github.com/CoreSenseEU/understanding-logic)
- [coresense_msgs](https://github.com/CoreSenseEU/coresense_msgs)
- [coresense_vampire](https://github.com/CoreSenseEU/coresense_vampire)
- [triplestar_kb](https://github.com/CoreSenseEU/triplestar_kb)

This should work in any ROS2 distribution.

### Preparation

#### Making your skills available
1. Create BTs for each skill inside your ROS nodes like this:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <!-- Aggregate Meta -->
  <BehaviorTree ID="MakeSureAlternativesEvaluated_Template">
    <Sequence _successIf="evaluation_last_updated_iteration >= iteration">
      <SubTree ID="MakeSureAlternativesAssessed_Template" _autoremap="true" />
      <Aggregate assessments="{assessments}"
                 evaluation="{evaluation}"
                 action_name=""/>
      <Script code="evaluation_last_updated_iteration = iteration" />
    </Sequence>
  </BehaviorTree>
</root>
```

2. Annotate Your ROS nodes like this:
```json
{
  "name": "aggregate_utility_boolean_engine",
  "inputs": [
    {
      "name": "assessment_matrix_2_template",
      "formalism" : "assessment_matrix_msg",
      "representation_classes": ["assessment_matrix_rc"],
      "concepts": [],
      "creator" : "",
      "requirements": []
    }
  ],
  "engine_output": {
    "name": "evaluation_modelet",
    "formalism" : "evaluation_msg",
    "representation_classes": ["evaluation_rc"],
    "concepts": [],
    "properties": [
      {
        "name": "num_axes_property",
        "datatype": "unsigned_datatype",
        "value": "1"
      },
      {
        "name": "entry_representation_class_property",
        "datatype": "representation_class_datatype",
        "value": "boolean"
      }
    ]
  },
  "transit_properties": [],
  "time_delay": 0,
  "energy_cost": 0.0,
  "resources_consumed": [],
  "resources_blocked": [
    {
      "name": "cpu_resource",
      "percentage": 0.0
    }
  ]
}
```
This should be in a ROS parameter called `coresense_engine`.

We have a set of example skills at: [coresense_understanding_examples](https://github.com/CoreSenseEU/coresense_understanding_examples)

### Usage

#### Via launch file


1. Run the understanding system
```bash
ros2 launch coresense_understanding_bringup coresense_understanding.launch.py
```

2. Start a session
```bash
ros2 service call /understanding/start_session coresense_msgs/srv/StartSession "{}"
export SESSION_ID=[replace with id returned by the service call]
```

3. Ask for a strategy
```bash
export TPTP=$(cat query.tff)
ros2 action send_goal /understanding/understand coresense_msgs/action/Understand "{session_id: '$SESSION_ID', target_modelet: '$TPTP'}"
```

4. End the session
```bash
ros2 service call /understanding/end_session coresense_msgs/srv/EndSession "{session_id: '$SESSION_ID'}"
```
