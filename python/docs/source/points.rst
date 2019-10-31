Points
======

Types
-----

.. autoclass:: cepton_sdk.Points
  :members:

.. autofunction:: cepton_sdk.combine_points

All point array classes support numpy indexing and assignment as if they were 1-d arrays:

.. code-block:: python
  :linenos:

  n_points = len(points_1)
  points_2[10:20] = points_1[:10]

Multiple point arrays can also be combined:

.. code-block:: python
  :linenos:

  points = cepton_sdk.combine_points([points_1, points_2])

Methods
-------

See :doc:`samples/listen`.

The following methods return points directly from the C SDK callback.

.. autofunction:: cepton_sdk.listen_frames
.. autofunction:: cepton_sdk.unlisten_frames

There are also listener classes that seamlessly handle accumulation and waiting.

.. autoclass:: cepton_sdk.FramesListener
.. autoclass:: cepton_sdk.SensorFramesListener

Export
------

.. autofunction:: cepton_sdk.export.save_points_las
.. autofunction:: cepton_sdk.export.load_points_las
.. autofunction:: cepton_sdk.export.save_points_ply
.. autofunction:: cepton_sdk.export.save_points_pcd
