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

Le dict `SCENARIOS` de `sim.py` contient plusieurs autres cartes/ratios
robots-victimes/pannes (voir [Scénarios](#scénarios) plus bas) - chacun a
un jumeau paramétriquement identique sur les branches décentralisées,
pour pouvoir comparer directement quelle architecture convient le mieux
plutôt que d'en discuter dans l'abstrait.

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

## Scénarios

`python3 sim.py --scenario NOM` lance l'un des scénarios suivants au lieu
du scénario par défaut (un nom invalide liste les noms valides dans le
message d'erreur) :

| nom | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots, 4 victimes, 15×15, une panne scriptée | la démo de base ci-dessus |
| `many_victims` | mêmes robots/carte, 9 victimes | tenue de l'appariement glouton quand les victimes sont plus nombreuses que les robots |
| `robot_heavy` | 6 robots, 2 victimes | sur-provisionnement / comportement des robots inactifs |
| `large_map` | carte 25×25, 5 robots, 7 victimes, une panne | passage à l'échelle sur une carte et un essaim plus grands |
| `double_failure` | les robots 1 et 0 tombent tous deux en panne (ticks 11 et 30) | résilience quand l'essaim perd la majorité de sa capacité et que le Coordinator doit réassigner deux fois |
| `short_sensors` | `sensor_range` environ divisé par deux (1.2) | l'importance de la portée de perception quand le Coordinator ne peut assigner que ce qui a été signalé |

`python3 sim.py --compare` lance tous les scénarios sans rendu (pas de
GIF/fenêtre) et affiche un tableau du nombre de ticks jusqu'à
complétion — c'est aussi le test de bout en bout du projet : un scénario
qui plante ou ne termine jamais (DNF) avant son `max_ticks` est un vrai
bug. Les paramètres de chaque scénario sont identiques à son jumeau sur
les branches décentralisées, donc la sortie de `--compare` des deux
branches est faite pour être lue côte à côte, pas seulement au sein d'une
branche. Sur les scénarios testés jusqu'ici, l'allocation centralisée
termine nettement plus vite quand l'essaim est en bonne santé
(`default` : 87 ticks ici contre 139 en décentralisé) car l'appariement
glouton du Coordinator est globalement optimal plutôt que
premier-détecté-premier-réclamé, mais perd vite cet avantage sous
`double_failure` (172 ticks ici contre 144 en décentralisé), car perdre
deux robots sur trois laisse l'avantage d'allocation du Coordinator avec
un seul robot pour l'exécuter.

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
