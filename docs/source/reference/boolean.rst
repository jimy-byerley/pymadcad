.. _boolean:

boolean    - Boolean cut/intersect/stitch meshes
================================================

.. automodule:: madcad.boolean

Most common
-----------

.. autofunction:: pierce

	Between `Web`  (The result is the white part, the green part is the `ref` parameter)
	
	.. code-block::
	
		rect = web(wire([
			vec3(-w,-h,0), 
			vec3(w,-h,0), 
			vec3(w,h,0), 
			vec3(-w,h,0),
			]).close().segmented())
		hole = web(Circle((O,Z), 1.5))
		
		result = pierce(rect, hole)
	
	.. image:: /screenshots/pierce-web-before.png
	.. image:: /screenshots/pierce-web.png
	
	Between `Web` and `Mesh`
	
	.. code-block::
	
		rect = web(wire([
			vec3(-w,-h,0), 
			vec3(w,-h,0), 
			vec3(w,h,0), 
			vec3(-w,h,0),
			]).close().segmented())
		hole = extrusion(4*Z, Circle((O,Z), 1.5), alignment=0.5)
		
		result = pierce(rect, hole)
	
	.. image:: /screenshots/pierce-web-mesh-before.png
	.. image:: /screenshots/pierce-web-mesh.png
	
	Between `Mesh`
	
	.. code-block::
	
		rect = flatsurface(wire([
			vec3(-w,-h,0), 
			vec3(w,-h,0), 
			vec3(w,h,0), 
			vec3(-w,h,0),
			]))
		rect = (  rect.transform(Z) 
		        + rect.transform(-Z).flip() )
		hole = extrusion(4*Z, 
			flatsurface(Circle((O,Z), 1.5)).flip(), 
			alignment=0.5)
		
		result = pierce(rect, hole)
	
	.. image:: /screenshots/pierce-mesh-before.png
	.. image:: /screenshots/pierce-mesh.png

.. autofunction:: boolean

	Between `Web`
	
	.. code-block::
	
		w, h = 2, 1
		a = web(wire([
		        vec3(-w,-h,0), 
		        vec3(w,-h,0), 
		        vec3(w,h,0), 
		        vec3(-w,h,0),
		        ]).close().segmented())
		b = web(Circle((O,Z), 1.5))
		
		result = boolean(a, b, (False,False))
	
	.. image:: /screenshots/boolean-boolean-web-before.png
	.. image:: /screenshots/boolean-boolean-web.png
	
	Between `Mesh`
	
	.. code-block::
	
		w, h = 3, 2
		rect = flatsurface(wire([
		          vec3(-w,-h,0), 
		          vec3(w,-h,0), 
		          vec3(w,h,0), 
		          vec3(-w,h,0),
		          ]))
		a = (  rect.transform(Z)
		    +  rect.transform(-Z).flip() )
		b = extrusion(4*Z, 
		        flatsurface(Circle((O,Z), 1.5)).flip(), 
		        alignment=0.5)
		
		result = boolean(a, b, (False,False))
	
	.. image:: /screenshots/boolean-boolean-mesh-before.png
	.. image:: /screenshots/boolean-boolean-mesh.png


Those are shortcuts for `boolean`:

.. autofunction:: union

	.. image:: /screenshots/boolean-union.png
	
.. autofunction:: difference

	.. image:: /screenshots/boolean-difference.png

.. autofunction:: intersection

	.. image:: /screenshots/boolean-intersection.png

More advanced
-------------

.. autofunction:: cut_web

	.. image:: /screenshots/boolean-cut-web-before.png
	.. image:: /screenshots/boolean-cut-web.png

.. autofunction:: cut_web_mesh

	.. image:: /screenshots/boolean-cut-web-mesh-before.png
	.. image:: /screenshots/boolean-cut-web-mesh.png

.. autofunction:: cut_mesh

	.. image:: /screenshots/boolean-cut-mesh-before.png
	.. image:: /screenshots/boolean-cut-mesh.png
