﻿using OpenH2.Core.Maps;
using OpenH2.Core.Tags.Layout;
using OpenH2.Serialization.Layout;

namespace OpenH2.Core.Tags
{
    [TagLabel(TagName.mach)]
    public class MachineryTag : BaseTag
    {
        public override string Name { get; set; }
        public MachineryTag(uint id) : base(id)
        {
        }

        [PrimitiveValue(0)]
        public int Type { get; set; }

        [PrimitiveValue(4)]
        public float UniformScale { get; set; }

        [PrimitiveValue(16)]
        public float Unknown { get; set; }

        [PrimitiveValue(56)]
        public TagRef<HaloModelTag> Model { get; set; }

        [PrimitiveValue(64)]
        public TagRef<BlocTag> Bloc { get; set; }

        [PrimitiveValue(80)]
        public uint EffectId { get; set; }

        [PrimitiveValue(88)]
        public uint FootId { get; set; }

        [PrimitiveValue(196)]
        public float ValueA { get; set; }

        [PrimitiveValue(200)]
        public float PositionChangeRate { get; set; }

        [PrimitiveValue(204)]
        public float UnknownChangeRate { get; set; }

        [PrimitiveValue(212)]
        public float ValueD { get; set; }

        [PrimitiveValue(280)]
        public float ActivationRange { get; set; }
    }
}
