# Essaim décentralisé de recherche & sauvetage (démo)

Démo autonome en Python pur (pas de ROS2/Nav2/Gazebo requis) pour valider un
principe d'architecture avant de l'implémenter sur une vraie stack robotique.

## Ce que ça démontre

Le scénario par défaut : 3 robots terrestres partent de coins opposés
d'une carte 15×15 avec 4 victimes cachées. Chaque robot exécute son
propre arbre de comportement et ne connaît que sa position, ce que ses
capteurs détectent, et ce que sa radio a reçu — aucun robot n'a jamais
vue sur tout le plateau. Au tick 11, le robot 1 est scripté pour tomber
en panne juste après avoir réclamé une victime. Les deux autres
remarquent que la réclamation est périmée et reprennent la mission sans
intervention extérieure — c'est tout l'intérêt de la démo : une
coordination qui *émerge* de règles locales, pas d'un ordonnanceur.

Le dict `SCENARIOS` de `sim.py` contient plusieurs autres cartes/ratios
robots-victimes/pannes (voir [Scénarios](#scénarios) plus bas) - chacun a
un jumeau paramétriquement identique sur la branche `centralized-swarm`,
pour pouvoir comparer directement quelle architecture convient le mieux
plutôt que d'en discuter dans l'abstrait.

## Comment c'est structuré

- **`swarm/robot.py`** (classes de feuilles + `Robot._build_tree()`) —
  l'arbre est construit avec [py_trees](https://py-trees.readthedocs.io/)
  (`Selector`, `Sequence`, une sous-classe `py_trees.behaviour.Behaviour`
  par feuille), avec les mêmes noms de nœuds que la taxonomie de
  BehaviorTree.CPP / Nav2 (`nav2_behavior_tree`). Chaque robot exécute son
  propre arbre, tické une fois par pas de temps.
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
pip install -r requirements.txt
python3 sim.py
```

Sortie : log texte des décisions (qui réclame quoi, qui secourt qui, la
panne scriptée) + `output/sar_swarm_demo.gif`. L'animation est un tableau
de bord : la carte (avec une flèche de déplacement par robot) et l'arbre
de comportement complet de chaque robot, chaque nœud coloré selon son
statut py_trees en direct.

Ajoutez `--live` pour ouvrir une fenêtre matplotlib interactive qui joue
automatiquement au lieu de seulement sauvegarder le GIF, ou `--slider`
pour la même fenêtre en pause avec un curseur de tick et un bouton
Play/Pause pour naviguer à la main (les deux nécessitent un affichage) :

```bash
python3 sim.py --live
python3 sim.py --slider
```

## Scénarios

`python3 sim.py --scenario NOM` lance l'un des scénarios suivants au lieu
du scénario par défaut (un nom invalide liste les noms valides dans le
message d'erreur) :

| nom | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots, 4 victimes, 15×15, une panne scriptée | la démo de base ci-dessus |
| `many_victims` | mêmes robots/carte, 9 victimes | contention des réclamations quand les victimes sont plus nombreuses que les robots |
| `robot_heavy` | 6 robots, 2 victimes | sur-provisionnement / comportement des robots inactifs |
| `large_map` | carte 25×25, 5 robots, 7 victimes, une panne | passage à l'échelle sur une carte et un essaim plus grands |
| `double_failure` | les robots 1 et 0 tombent tous deux en panne (ticks 11 et 64) | résilience quand l'essaim perd la majorité de sa capacité |
| `short_sensors` | `sensor_range` environ divisé par deux (1.2) | l'importance de la portée de perception quand la réclamation est arbitrée entre pairs |

`python3 sim.py --compare` lance tous les scénarios sans rendu (pas de
GIF/fenêtre) et affiche un tableau du nombre de ticks jusqu'à
complétion — c'est aussi le test de bout en bout du projet : un scénario
qui plante ou ne termine jamais (DNF) avant son `max_ticks` est un vrai
bug.

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
   nœuds `PickExploreGoal`/`ClaimVictim`/`NavigateToPose` de ce prototype
   deviendraient des `BT.CPP` nodes appelant l'action `NavigateToPose` de
   Nav2 au lieu de déplacer un point sur une grille — voir la branche
   `nav2-port`, qui fait exactement cela avec les mêmes noms de nœuds BT.
2. **Radio** : `radio_range` est ici volontairement large (quasi tout la
   carte) pour rester simple. Sur ROS2, le bus serait un topic DDS
   (`/swarm/claims`) avec QoS *best-effort* — DDS gère nativement la
   découverte pair-à-pair sans master central, ce qui correspond
   exactement à l'hypothèse "pas d'arbitre" de ce prototype.

La logique de réclamation/timeout/reprise sur panne, elle, se porte telle
quelle : c'est la partie qui valide réellement le fonctionnement
décentralisé, indépendamment de la stack de navigation utilisée en dessous.
