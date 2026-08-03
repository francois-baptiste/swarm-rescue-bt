# Essaim décentralisé de recherche & sauvetage (démo)

Démo autonome en Python pur (pas de ROS2/Nav2/Gazebo requis) pour valider un
principe d'architecture avant de l'implémenter sur une vraie stack robotique :

- **`bt/core.py`** — moteur de behavior tree fait maison (Sequence, Selector,
  Condition, Action), calqué sur la taxonomie de BehaviorTree.CPP / Nav2
  (`nav2_behavior_tree`). Chaque robot exécute son propre arbre, tické une
  fois par pas de temps.
- **`swarm/world.py`** — grille 2D avec obstacles + BFS, qui joue le rôle du
  planificateur global + contrôleur local de Nav2 (donner un chemin vers un
  objectif, avancer d'une cellule).
- **`swarm/comms.py`** — bus radio simulé (broadcast à portée limitée, un
  tick de latence). Aucun arbitre : ce fichier ne décide jamais rien, il
  ne fait que propager des messages.
- **`swarm/robot.py`** — un robot = un arbre de comportement + un état
  purement local (position, ce qu'il a vu, ce qu'il a entendu). La
  coordination émerge de la règle "je réclame ce que je vois et qui n'est
  pas déjà activement réclamé" + "une réclamation sans nouvelle depuis
  15 ticks est considérée périmée" (tolérance de panne, type contract-net
  simplifié).
- **`sim.py`** — scénario : 3 robots terrestres partent de coins opposés
  d'une carte 15x15 avec 4 victimes cachées. Au tick 11, le robot 1 est
  scripté pour tomber en panne juste après avoir réclamé une victime, afin
  de montrer que les deux autres robots reprennent la mission sans
  intervention extérieure.

## Lancer la démo

```bash
python3 sim.py
```

Sortie : log texte des décisions (qui réclame quoi, qui secourt qui, la
panne scriptée) + `output/sar_swarm_demo.gif` (animation matplotlib).

## Pourquoi c'est décentralisé

Aucun processus n'a de vue globale des réclamations ni n'arbitre les
conflits. Chaque robot ne connaît que : sa position, ce que ses "capteurs"
détectent dans son `sensor_range`, et ce que la radio lui a livré. Retirer
un robot de la liste dans `sim.py` ne casse rien chez les deux autres — la
panne scriptée du robot 1 le démontre directement dans le scénario.

## Vers une vraie stack Nav2 multi-robot

Ce prototype simplifie deux choses pour rester lisible :

1. **Navigation** : BFS sur grille connue remplace ici Nav2
   (`bt_navigator` + `planner_server` + `controller_server`). Sur une vraie
   stack, chaque robot aurait son propre namespace ROS2
   (`/robot_0/...`, `/robot_1/...`) avec sa propre instance Nav2, et les
   nœuds `Explore`/`ClaimAndBroadcast`/`NavigateToClaim` de ce prototype
   deviendraient des `BT.CPP` nodes appelant l'action `NavigateToPose` de
   Nav2 au lieu de déplacer un point sur une grille.
2. **Radio** : `radio_range` est ici volontairement large (quasi tout la
   carte) pour rester simple. Sur ROS2, le bus serait un topic DDS
   (`/swarm/claims`) avec QoS *best-effort* — DDS gère nativement la
   découverte pair-à-pair sans master central, ce qui correspond
   exactement à l'hypothèse "pas d'arbitre" de ce prototype.

La logique de réclamation/timeout/reprise sur panne, elle, se porte telle
quelle : c'est la partie qui valide réellement le fonctionnement
décentralisé, indépendamment de la stack de navigation utilisée en dessous.
