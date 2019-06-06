﻿using OpenH2.Core.Tags.Layout;

namespace OpenH2.Core.Tags
{
    [TagLabel("scnr")]
    public class Scenario : BaseTag
    {
        public Scenario(uint id) : base(id)
        {
        }

        public override string Name { get; set; }

        [InternalReferenceValue(8)]
        public SkyboxReference[] SkyboxReferences { get; set; }

        [InternalReferenceValue(528)]
        public Terrain[] Terrains { get; set; }

        [FixedLength(8)]
        public class SkyboxReference
        {
            [PrimitiveValue(4)]
            public uint SkyboxId { get; set; }
        }

        [FixedLength(68)]
        public class Terrain
        {
            [PrimitiveValue(20)]
            public uint BspId { get; set; }
            public Bsp Bsp { get; set; }

            // TODO implement lightmap tag
            [PrimitiveValue(28)]
            public uint LightmapId { get; set; }
            public BaseTag Lightmap { get; set; }
        }
    }
}
