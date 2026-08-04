# swarm-rescue-bt (branche centralisée)

Cinq simulations d'essaim de robots centralisées, en Python pur (pas de
ROS2/Nav2/Gazebo requis). Chacune valide un principe d'architecture — des
behavior trees pour la perception et la navigation locales, avec un
Coordinator unique qui prend toutes les décisions d'allocation — avant de
l'implémenter sur une vraie stack robotique. À comparer avec les branches
`py-trees-port` / `master`, où les mêmes missions sont résolues sans
aucun arbitre central.

![demo](output/sar_swarm_demo.gif)

## Ce que ça démontre

La mission phare (`rescue`, la démo d'origine) : 3 robots terrestres
partent de coins opposés d'une carte 15×15 avec 4 victimes cachées.
Chaque robot exécute toujours son propre arbre de comportement et ne
perçoit toujours que les victimes dans son propre `sensor_range` — mais
au lieu de réclamer une victime détectée pour lui-même, il la signale
(avec un heartbeat) à un unique `swarm.coordinator.Coordinator`, seul à
décider quel robot poursuit quelle victime. Au tick 11, le robot 1 est
scripté pour tomber en panne juste après s'être vu assigner une victime.
Le Coordinator remarque le silence et réassigne la victime à un autre
robot — l'idée de *cette* démo est que l'allocation est simple et
globalement optimale tant que le Coordinator est vivant, au prix d'un
point unique de défaillance pour toute l'allocation de tâches de
l'essaim.

Quatre autres missions (voir [Missions](#missions) plus bas) explorent la
même idée sous d'autres formes — patrouiller un périmètre partagé, tenir
une chaîne de relais radio, chasser une cible mobile, et un
capture-the-flag à deux équipes (la seule mission où un Coordinator n'a
rien à décider différemment — voir sa section dédiée plus bas). Chaque
mission a un jumeau paramétriquement identique sur les branches
décentralisées, pour pouvoir comparer directement quelle architecture
convient le mieux plutôt que d'en discuter dans l'abstrait.

## Comment c'est structuré

- **`missions/<nom>.py`** — un fichier par mission (`rescue`, `patrol`,
  `relay`, `wolfpack`, `capture_flag`), chacun définissant son propre
  Coordinator (là où il en a un), sa/ses classe(s) de robot, son dict
  `SCENARIOS`, `build_scenario`/`run`/`print_log`/`summarize`, et les
  décorations de carte dont `sim.py` a besoin pour l'afficher
  (`draw_extra`/`legend_handles`). `missions/common.py` contient ce que
  toutes les missions partagent (la palette de couleurs, l'aide py_trees
  pour capturer l'état de l'arbre). Le Coordinator de `rescue` vit dans
  `swarm/coordinator.py` à la place, car il précède le système de
  missions et d'autres fichiers l'importent encore directement.
- L'arbre de chaque mission est construit avec
  [py_trees](https://py-trees.readthedocs.io/) (`Selector`, `Sequence`,
  une sous-classe `py_trees.behaviour.Behaviour` par feuille), avec la
  même taxonomie de noms que BehaviorTree.CPP / Nav2
  (`nav2_behavior_tree`) — `NavigateToPose` veut dire la même chose
  (marcher une cellule vers un objectif via BFS) dans chaque mission,
  quelle que soit la feuille qui décide de cet objectif.
- **`swarm/world.py`** — grille 2D avec obstacles + BFS, qui joue le rôle
  du planificateur global + contrôleur local de Nav2 (donner un chemin
  vers un objectif, avancer d'une cellule).
- **`sim.py`** — pilote agnostique de la mission : parsing CLI, la
  mécanique partagée de rendu/GIF/`--live`/`--slider`/`--compare`, qui
  distribue vers le module `missions/<nom>.py` sélectionné par
  `--mission`.

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

## Missions

`python3 sim.py --mission NOM` choisit une mission (par défaut `rescue`) ;
`--scenario NOM` choisit ensuite un des scénarios de cette mission
(`--mission`/`--scenario` acceptent un nom invalide aussi, et listent les
noms valides dans leur message d'erreur). `python3 sim.py --mission NOM
--compare` lance tous les scénarios de cette mission sans rendu (pas de
GIF/fenêtre) et affiche un tableau comparatif — c'est aussi le test de
bout en bout du projet : un scénario qui plante ou ne termine jamais
(DNF) avant son `max_ticks` est un vrai bug. Les paramètres de chaque
scénario sont identiques à son jumeau sur les branches décentralisées,
donc la sortie de `--compare` des deux branches est faite pour être lue
côte à côte.

### `rescue` (par défaut)

Chercher des victimes sur une grille et les secourir ; un Coordinator
apparie glouton chaque victime connue et non assignée à son robot inactif
le plus proche. Voir [Ce que ça démontre](#ce-que-ça-démontre) plus haut.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots, 4 victimes, 15×15, une panne scriptée | la démo de base |
| `many_victims` | mêmes robots/carte, 9 victimes | tenue de l'appariement glouton quand les victimes sont plus nombreuses que les robots |
| `robot_heavy` | 6 robots, 2 victimes | sur-provisionnement / comportement des robots inactifs |
| `large_map` | carte 25×25, 5 robots, 7 victimes, une panne | passage à l'échelle sur une carte et un essaim plus grands |
| `double_failure` | les robots 1 et 0 tombent tous deux en panne (ticks 11 et 30) | résilience quand l'essaim perd la majorité de sa capacité et que le Coordinator doit réassigner deux fois |
| `short_sensors` | `sensor_range` environ divisé par deux (1.2) | l'importance de la portée de perception quand le Coordinator ne peut assigner que ce qui a été signalé |

Sur les scénarios testés jusqu'ici, l'allocation centralisée termine
nettement plus vite quand l'essaim est en bonne santé (`default` :
87 ticks ici contre 139 en décentralisé) car l'appariement glouton du
Coordinator est globalement optimal plutôt que
premier-détecté-premier-réclamé, mais perd vite cet avantage sous
`double_failure` (172 ticks ici contre 144 en décentralisé), car perdre
deux robots sur trois laisse l'avantage d'allocation du Coordinator avec
un seul robot pour l'exécuter.

### `patrol`

N robots divisent une boucle de périmètre partagée en arcs contigus.
Contrairement aux branches décentralisées, la répartition n'est pas
calculée par chaque robot lui-même : un Coordinator possède la boucle et
rééquilibre les arcs entre les robots encore actifs (heartbeat) à chaque
changement de cet ensemble.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 4 robots patrouillent une bordure 15×15, aucune panne | couverture complète |
| `robot_down` | le robot 1 tombe en panne presque immédiatement (tick 3) | le Coordinator remarque (après `FAILURE_TIMEOUT`) et rééquilibre la boucle entre les 3 survivants — **la couverture complète est récupérée** (52/52), contrairement à l'écart permanent de 43/52 de la branche décentralisée |

### `relay`

N robots tiennent des positions régulièrement espacées entre une source
et un puits fixes pour que chaque saut consécutif reste dans
`comm_range`. Le Coordinator possède la répartition des slots et la
rééquilibre entre les survivants en cas de panne — mais réétaler ce que
N robots couvraient sur moins de robots ne rentre pas toujours dans
`comm_range`.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots tiennent une chaîne coin à coin | la chaîne reste intacte |
| `robot_down` | le robot relais du milieu tombe en panne une fois installé | le Coordinator réétale les 2 survivants sur tout le trajet, **réduisant le pire écart de 10.0 (branche décentralisée) à 7.07** — mieux, mais toujours au-dessus de `comm_range=6.0`, car 2 robots ne peuvent physiquement pas couvrir ce que 3 couvraient |

### `wolfpack`

N robots chassent une proie évasive. Contrairement aux branches
décentralisées, un robot qui repère la proie ne diffuse pas à ses
pairs — il signale directement à un Coordinator, et chaque robot
interroge ce même Coordinator pour le signalement le plus récent, sans
délai de propagation ni question de portée radio.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots chassent 1 proie, aucune panne | une chasse directe |
| `robot_down` | le robot qui repère la proie en premier tombe en panne juste après l'avoir signalée | le signalement a déjà atteint le Coordinator, donc les autres robots agissent dessus au tick suivant — en pratique les chiffres de cette mission restent proches de ceux de la branche décentralisée, sa portée radio généreuse faisant rarement du délai de propagation le facteur limitant |

### `capture_flag`

Deux équipes (rouge/bleue) courent toucher le drapeau ennemi ; un robot
surpris à moins de `tag_range` d'un robot ennemi alors qu'il est du côté
de la carte de cet ennemi réapparaît à son point de départ. Les positions
des drapeaux sont de notoriété publique (pas de phase de détection), donc
il n'y a aucune tâche à allouer — **cette mission n'a aucun Coordinator**,
et elle est identique à sa jumelle décentralisée pour cette raison
précise (même code, mêmes résultats vérifiés).

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 contre 3, départs symétriques | un combat équilibré |
| `outnumbered` | rouge (2) contre bleu (5) | plus d'attaquants *et* plus de défenseurs incidents |

## Pourquoi c'est centralisé

Le Coordinator de chaque mission (`swarm/coordinator.py` pour `rescue`,
une petite classe locale à chaque autre `missions/<nom>.py`) est le seul
processus à avoir une vue globale de l'état de l'essaim et à arbitrer qui
fait quoi. Les robots ne se parlent jamais entre eux et ne décident
jamais l'allocation eux-mêmes — retirez le Coordinator et chaque robot
continue de percevoir, d'explorer et de se déplacer, mais plus rien
n'est jamais assigné, réparti, ou (pour `rescue`) secouru. C'est
l'inverse délibéré du "retirer n'importe quel robot ne casse rien chez
les autres" des branches décentralisées — voir leurs README pour le
compromis dans l'autre sens, et `patrol`/`relay` ci-dessus pour ce que la
centralisation rend en échange : le travail d'un robot mort ne
disparaît pas, il est réparti sur ceux qui restent, dans la limite de ce
qu'ils peuvent physiquement couvrir.

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
