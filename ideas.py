
class GroupDef:
	qualifiers = set()
	attributes = dict()

body.group(Designate('softneck & junction | detail'))   un designate selectionne plus finement les groupes
body.group(lambda g:  'softneck' in g and 'junction' in g or 'detail' in g)

select(body, qualified('softneck') | groupcheck(lambda g: g['attribute'] > 5) & closest(P), stopangle(pi/8) | straight)
select(body, body.qualified('softneck') | body.closest(P), stopangle(pi/8) | straight)
body.select(qualified('softneck') & closest(P), stopangle(pi/8) | straight)



body.owngroups().mergegroups('newgroup', 'oldgroup')
body.ownpoints().strippoints()
body.own_groups().merge_groups('newgroup', 'oldgroup')
body.own_points().strip_points()

body.strippoints() -> new Self, reindex
body.mergepoints() -> new Self
body.mergeclose() -> new Self, reindex
body.stripgroups() -> new Self, reindex

body.istrippoints() -> reindex
body.imergepoints() -> reindex
body.istripgroups() -> reindex




minkowski(a, b) -> Mesh
brush(pattern, path, directions=None) -> Mesh
expand(surface, offset) -> Mesh
hull(mesh, concave=0, width=0) -> mesh
chamfer(mesh, edges, cutter, grouplimit=False, tangent_limit=True) -> MeshGroup


bolt(a, b, rscrew, washera=False, washerb=False)	# bolt assembly
sphere_plane_bolted_junction(axis, radius, rscrew)     # many bolts, soft surface between sphere and plane
bearing_slot_exterior(axis, rext, shouldering=True, circlip=False)   # revolution surface to place a bearing in
bearing_slot_interior(axis, rext, shouldering=True, circlip=False)   # revolution surface to place a bearing on
screw_slot(axis, rscrew, rslot=None, hole=True, expand=None)  # revolution surface to place a bolt in
bolt_slot(a, b, rscrew, rslot=None, hole=True, expanda=None, expandb=None)  # revolution surface to place a bolt in

Solid.globalref('sub', 'element', 'subgroup')  # return it all transformed
Solid.localref('sub', 'element', 'subgroup')  # return it not transformed, as with __getitem__
