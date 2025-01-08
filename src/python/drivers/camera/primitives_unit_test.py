import geometry

from drivers.camera import primitives

_EXAMPLE_OBJECTS = [
    primitives.TrackedObjects(
        id=0,
        label=primitives.ObjectType.PERSON,
        position=geometry.Position(
            frame=geometry.FRONT_CAMERA,
            x=-0.21148939430713654,
            y=0.4225114583969116,
            z=-1.7230194807052612,
        ),
        velocity=geometry.Velocity(
            frame=geometry.FRONT_CAMERA,
            x=-0.01301808562129736,
            y=0.004759322851896286,
            z=-0.0016298347618430853,
        ),
        image_bounding_box=((100, 100), (300, 300)),
        bounding_box=[
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=-0.849174439907074,
                y=0.7171362042427063,
                z=-1.089142084121704,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=-0.8491111397743225,
                y=0.6935141086578369,
                z=-2.364882230758667,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=0.42684340476989746,
                y=0.6969338655471802,
                z=-2.364882230758667,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=0.426780104637146,
                y=0.7205560207366943,
                z=-1.089142084121704,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=-0.8476551175117493,
                y=0.15027129650115967,
                z=-1.0786457061767578,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=-0.8475918173789978,
                y=0.12664920091629028,
                z=-2.3543858528137207,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=0.42836272716522217,
                y=0.13006898760795593,
                z=-2.3543858528137207,
            ),
            geometry.Position(
                frame=geometry.FRONT_CAMERA,
                x=0.4282994270324707,
                y=0.15369108319282532,
                z=-1.0786457061767578,
            ),
        ],
        width=1.2759590148925781,
        height=0.5669640898704529,
        length=1.275958776473999,
        confidence=93.45703125,
    )
]


def test_primitives() -> None:
    ...
