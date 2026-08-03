# swarm-rescue-bt

Recherche & sauvetage décentralisée par un essaim de robots, construite
sur Nav2 : un arbre de comportement de mission par robot (BehaviorTree.CPP
v3, même taxonomie que `nav2_behavior_tree` de Nav2) qui réutilise le
nœud `NavigateToPose` livré par Nav2 pour la navigation, tout en
coordonnant l'essaim de pair à pair via des topics ROS2 — aucun arbitre
central nulle part dans la stack.

Le package se trouve dans
[`ros2_ws/src/swarm_sar_bt/`](ros2_ws/src/swarm_sar_bt/) — voir son
README pour l'architecture, les instructions de build/run, et les
simplifications connues (ce qui est vérifié ou non, ce qu'une version de
production devrait encore changer).

---

*[English version](README.md)*
