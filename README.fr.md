# Essaim centralisé de recherche & sauvetage (branche centralisée)

Démo autonome en Python pur (pas de ROS2/Nav2/Gazebo requis) pour valider
un principe d'architecture — des behavior trees pour la perception et la
navigation locales, avec un Coordinator unique qui prend toutes les
décisions d'allocation de tâches — avant de l'implémenter sur une vraie
stack robotique. À comparer avec les branches `py-trees-port` / `master`,
où le même scénario est résolu sans aucun arbitre central.

![demo](output/sar_swarm_demo.gif)

## Ce que ça démontre

3 robots terrestres partent de coins opposés d'une carte 15×15 avec
4 victimes cachées. Chaque robot exécute toujours son propre arbre de
comportement et ne perçoit toujours que les victimes dans son propre
`sensor_range` — mais au lieu de réclamer une victime détectée pour
lui-même, il la signale (avec un heartbeat) à un unique
`swarm.coordinator.Coordinator`, seul à décider quel robot poursuit
quelle victime. Au tick 11, le robot 1 est scripté pour tomber en panne
juste après s'être vu assigner une victime. Le Coordinator remarque le
silence et réassigne la victime à un autre robot — l'idée de *cette* démo
est que l'allocation est simple et globalement optimale tant que le
Coordinator est vivant, au prix d'un point unique de défaillance pour
toute l'allocation de tâches de l'essaim.

## Comment c'est structuré

- **`swarm/coordinator.py`** — le Coordinator : un registre faisant
  autorité sur les victimes connues et les assignations en cours, un
  timeout de heartbeat qui réassigne la victime d'un robot silencieux, et
  un appariement glouton robot/victime le plus proche, exécuté une fois
  par tick après que chaque robot a perçu et bougé.
- **`swarm/robot.py`** (classes de feuilles + `Robot._build_tree()`) —
  l'arbre est construit avec [py_trees](https://py-trees.readthedocs.io/)
  (`Selector`, `Sequence`, une sous-classe `py_trees.behaviour.Behaviour`
  par feuille). `DetectVictim`/`ReportVictim` remplacent
  `DetectVictim`/`ClaimVictim` des branches décentralisées — un robot
  signale ce qu'il perçoit au lieu de décider de le réclamer — et
  `HasAssignment`/`PursueAssignment` remplacent `HasClaim`/`PursueClaim`.
  `Explore` est inchangé : le déplacement reste une décision purement
  locale même ici.
- **`swarm/world.py`** — grille 2D avec obstacles + BFS, qui joue le rôle
  du planificateur global + contrôleur local de Nav2 (donner un chemin
  vers un objectif, avancer d'une cellule).
- **`sim.py`** — le scénario décrit ci-dessus, qui assemble les trois
  robots, le monde et le Coordinator, et rend le run.

## Lancer la démo

```bash
pip install -r requirements.txt
python3 sim.py
```

Sortie : log texte des décisions du Coordinator (qui il assigne à quoi,
qui il réassigne et pourquoi, qui signale un secours) +
`output/sar_swarm_demo.gif`. L'animation est un tableau de bord : la
carte (avec une flèche de déplacement par robot) et l'arbre de
comportement complet de chaque robot, chaque nœud coloré selon son statut
py_trees en direct.

Ajoutez `--live` pour ouvrir une fenêtre matplotlib interactive qui joue
automatiquement au lieu de seulement sauvegarder le GIF, ou `--slider`
pour la même fenêtre en pause avec un curseur de tick et un bouton
Play/Pause pour naviguer à la main (les deux nécessitent un affichage) :

```bash
python3 sim.py --live
python3 sim.py --slider
```

## Pourquoi c'est centralisé

Le `Coordinator` de `swarm/coordinator.py` est le seul processus à avoir
une vue globale de chaque victime connue et de chaque assignation, et le
seul endroit qui arbitre les conflits et détecte un robot silencieux. Les
robots ne se parlent jamais entre eux et ne décident jamais qui poursuit
quoi — retirez le Coordinator et chaque robot continue de percevoir et
d'explorer, mais plus rien n'est jamais assigné ni secouru. C'est
l'inverse délibéré du "retirer n'importe quel robot ne casse rien chez
les autres" des branches décentralisées — voir leurs README pour le
compromis dans l'autre sens.

## Vers une vraie stack Nav2 multi-robot

Ce prototype simplifie les deux mêmes choses que les branches
décentralisées, pour la même raison (rester lisible) :

1. **Navigation** : BFS sur grille connue remplace ici Nav2
   (`bt_navigator` + `planner_server` + `controller_server`). Sur une
   vraie stack, chaque robot aurait sa propre instance Nav2 dans son
   propre namespace ROS2, et `PickExploreGoal`/`NavigateToPose` ici
   deviendraient des nœuds BT.CPP appelant l'action `NavigateToPose` de
   Nav2 au lieu de déplacer un point sur une grille.
2. **Liaison montante** : le signalement/heartbeat d'un robot vers le
   Coordinator est modélisé comme un appel de méthode instantané, sans
   délai de propagation (contrairement au tick de latence radio des
   branches décentralisées). Sur une vraie stack, ce serait un topic ou
   un appel de service ROS2 vers un nœud fleet-manager, qui a lui un vrai
   délai — le `FAILURE_TIMEOUT` du Coordinator devrait en tenir compte,
   comme le fait `CLAIM_TIMEOUT` sur les branches décentralisées.
