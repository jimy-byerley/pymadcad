

body = union(
		icosphere(O, rext)
			.name('centerball'),
		revolution(2*pi, Axis(O,Z), Softened([
				vec3(2.706, 0, 1.147),
				vec3(1.959, -1.56e-08, 2.209),
				vec3(1.221, -1.454e-07, 3.298),
				vec3(1.175, -4.856e-07, 6.152)])) .flip()
			.name('softneck'),
		) .transform(rint*0.3*Z)
		
body.unname('new')
chamfer(body, body.frontiers().select(closest(P), stopangle(pi/4)))
body.name('chamfer', 'new')

chamfer(body, body.frontiers().select(closest(P), stopangle(pi/4)), name='chamfer')
chamfer(body, body.frontiers().select(closest(P), stopangle(pi/4))).name('chanfer')

body.group('softneck')   -> extrait tous les groupes possedant cette designation
body.mergegroups('somenew')  -> fusionne tous les groupes en un nouveau donné par une designation
body.namegroups('somenew')
body.qual('somenew')
body.name('somenew')
body.name('somenew', replace='new')

Mesh.name(name, former=False)
Mesh.group(name)

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


