# gear -- Generation of gears, racks, etc

::: madcad.gear
    options:
      show_root_heading: false
      members: false

## Tooth profiles generation

The following functions are focussing on involute gears. If you want more details on how involutes gears are defined and computed, you can take a look at the [algorithm section](../algorithms/gearprofile.md)

::: madcad.gear.rackprofile
::: madcad.gear.gearprofile
::: madcad.gear.spherical_rackprofile
::: madcad.gear.spherical_gearprofile

## Gear generation

::: madcad.gear.gear
::: madcad.gear.geargather
::: madcad.gear.gearexterior
::: madcad.gear.gearstructure
::: madcad.gear.gearhub

## Bevel gear generation

::: madcad.gear.bevelgear

## Helper tools

::: madcad.gear.gearcircles
::: madcad.gear.involute
::: madcad.gear.involuteat
::: madcad.gear.involuteof
::: madcad.gear.spherical_involute
::: madcad.gear.spherical_involuteof

## Structure patterns

Those are the functions generating usual structures ready to used in `geargather`.

::: madcad.gear.pattern_circle
::: madcad.gear.pattern_full
::: madcad.gear.pattern_rect
::: madcad.gear.pattern_rounded
