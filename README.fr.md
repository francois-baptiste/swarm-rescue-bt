# swarm-rescue-bt

Cinq simulations d'essaim de robots décentralisées, en Python pur (pas de
ROS2/Nav2/Gazebo requis). Chacune valide un principe d'architecture — des
behavior trees pour des décisions purement locales, sans arbitre central —
avant de l'implémenter sur une vraie stack robotique.

## Ce que ça démontre

La mission phare (`rescue`, la démo d'origine) : 3 robots terrestres
partent de coins opposés d'une carte 15×15 avec 4 victimes cachées.
Chaque robot exécute son propre arbre de comportement et ne connaît que
sa position, ce que ses capteurs détectent, et ce que sa radio a reçu —
aucun robot n'a jamais vue sur tout le plateau. Au tick 11, le robot 1
est scripté pour tomber en panne juste après avoir réclamé une victime.
Les deux autres remarquent que la réclamation est périmée et reprennent
la mission sans intervention extérieure — c'est tout l'intérêt de la
démo : une coordination qui *émerge* de règles locales, pas d'un
ordonnanceur.

Quatre autres missions (voir [Missions](#missions) plus bas) explorent la
même idée sous d'autres formes — patrouiller un périmètre partagé, tenir
une chaîne de relais radio, chasser une cible mobile, et un
capture-the-flag à deux équipes. Chaque mission a un jumeau
paramétriquement identique sur la branche `centralized-swarm`, pour
pouvoir comparer directement quelle architecture convient le mieux
plutôt que d'en discuter dans l'abstrait.

## Comment c'est structuré

- **`missions/<nom>.py`** — un fichier par mission (`rescue`, `patrol`,
  `relay`, `wolfpack`, `capture_flag`), chacun définissant sa/ses
  classe(s) de robot, son dict `SCENARIOS`, `build_scenario`/`run`/
  `print_log`/`summarize`, et les décorations de carte dont `sim.py` a
  besoin pour l'afficher (`draw_extra`/`legend_handles`).
  `missions/common.py` contient ce que toutes les missions partagent (la
  palette de couleurs, l'aide py_trees pour capturer l'état de l'arbre).
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
- **`swarm/comms.py`** — bus radio simulé (broadcast à portée limitée, un
  tick de latence). Aucun arbitre : ce fichier ne décide jamais rien, il
  ne fait que propager des messages. Utilisé par `rescue` (réclamations)
  et `wolfpack` (signalements de la proie) ; `patrol`, `relay` et
  `capture_flag` n'ont besoin d'aucune radio, rien n'y étant décidé à
  partir de ce que dit un autre robot.
- **`sim.py`** — pilote agnostique de la mission : parsing CLI, la
  mécanique partagée de rendu/GIF/`--live`/`--slider`/`--compare`, qui
  distribue vers le module `missions/<nom>.py` sélectionné par
  `--mission`.

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

## Missions

`python3 sim.py --mission NOM` choisit une mission (par défaut `rescue`) ;
`--scenario NOM` choisit ensuite un des scénarios de cette mission
(`--mission`/`--scenario` acceptent un nom invalide aussi, et listent les
noms valides dans leur message d'erreur). `python3 sim.py --mission NOM
--compare` lance tous les scénarios de cette mission sans rendu (pas de
GIF/fenêtre) et affiche un tableau comparatif — c'est aussi le test de
bout en bout du projet : un scénario qui plante ou ne termine jamais
(DNF) avant son `max_ticks` est un vrai bug.

### `rescue` (par défaut)

Chercher des victimes sur une grille et les secourir ; l'allocation des
tâches émerge des réclamations diffusées sur le bus radio. Voir
[Ce que ça démontre](#ce-que-ça-démontre) plus haut.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots, 4 victimes, 15×15, une panne scriptée | la démo de base |
| `many_victims` | mêmes robots/carte, 9 victimes | contention des réclamations quand les victimes sont plus nombreuses que les robots |
| `robot_heavy` | 6 robots, 2 victimes | sur-provisionnement / comportement des robots inactifs |
| `large_map` | carte 25×25, 5 robots, 7 victimes, une panne | passage à l'échelle sur une carte et un essaim plus grands |
| `double_failure` | les robots 1 et 0 tombent tous deux en panne (ticks 11 et 64) | résilience quand l'essaim perd la majorité de sa capacité |
| `short_sensors` | `sensor_range` environ divisé par deux (1.2) | l'importance de la portée de perception quand la réclamation est arbitrée entre pairs |

### `patrol`

N robots divisent une boucle de périmètre partagée en arcs contigus,
calculés à partir du seul id du robot et du nombre total de robots — pas
de négociation nécessaire puisque chaque robot calcule la même répartition
indépendamment. Le prix de cette simplicité : si un robot meurt, personne
ne réassigne son arc, qui reste donc non patrouillé.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 4 robots patrouillent une bordure 15×15, aucune panne | couverture complète |
| `robot_down` | le robot 1 tombe en panne presque immédiatement (tick 3) | tout son arc reste non patrouillé pour toujours — l'assignation statique n'a pas de réallocation |

### `relay`

N robots tiennent des positions régulièrement espacées entre une source
et un puits fixes, pour que chaque saut consécutif reste dans
`comm_range`, relayant un message de bout en bout. Le slot de chaque
robot est une fonction fixe de son propre id — là encore, pas de
négociation, et là encore pas de réallocation si un robot meurt.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots tiennent une chaîne coin à coin | la chaîne reste intacte |
| `robot_down` | le robot relais du milieu tombe en panne une fois installé | la chaîne casse exactement là où il était — personne ne referme l'écart |

### `wolfpack`

N robots chassent une proie évasive qui fuit le robot le plus proche dès
qu'elle en remarque un. Un robot qui repère la proie diffuse le
signalement, et chaque robot converge vers le signalement le plus récent
qu'il a entendu, qu'il voie ou non la proie lui-même — la seule mission
ici où le partage pair-à-pair décentralisé est un véritable atout, pas un
handicap, puisque la mort d'un robot n'efface pas ce qu'il a déjà dit aux
autres.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 robots chassent 1 proie, aucune panne | une chasse directe |
| `robot_down` | le robot qui repère la proie en premier tombe en panne juste après l'avoir signalée | la piste survit — un autre robot reprend la chasse à partir du signalement diffusé |

### `capture_flag`

Deux équipes (rouge/bleue) courent toucher le drapeau ennemi ; un robot
surpris à moins de `tag_range` d'un robot ennemi alors qu'il est du côté
de la carte de cet ennemi réapparaît à son point de départ. Les positions
des drapeaux sont de notoriété publique (pas de phase de détection), donc
il n'y a aucune tâche à allouer — chaque robot, sur les deux
architectures, fonce toujours vers le drapeau ennemi. C'est la mission
des cinq qui a le moins de valeur comparative entre architectures, pour
cette raison précise : il n'y a rien qu'un Coordinator puisse décider
différemment.

| scénario | ce qui change | ce que ça teste |
| --- | --- | --- |
| `default` | 3 contre 3, départs symétriques | un combat équilibré |
| `outnumbered` | rouge (2) contre bleu (5) | plus d'attaquants *et* plus de défenseurs incidents |

## Pourquoi c'est décentralisé

Aucun processus n'a de vue globale de l'état de l'essaim ni n'arbitre les
conflits. Chaque robot ne connaît que sa position, ce que ses capteurs
détectent, et ce que la radio lui a livré. Retirer un robot de la liste
d'un scénario ne casse pas les autres — les scénarios de panne scriptée
de `rescue` et `wolfpack` le démontrent directement ; ceux de `patrol` et
`relay` démontrent le revers de la médaille : cette même absence de
coordinateur signifie aussi que personne ne réalloue le travail d'un
robot mort.

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
