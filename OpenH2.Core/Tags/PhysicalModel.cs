﻿using OpenH2.Core.Tags.Layout;
using System;
using System.Collections.Generic;
using System.Text;

namespace OpenH2.Core.Tags
{
    [TagLabel("hlmt")]
    public class PhysicalModelTag : BaseTag
    {
        public override string Name { get; set; }

        public PhysicalModelTag(uint id) : base(id)
        {
        }

        [PrimitiveValue(4)]
        public uint ModelId { get; set; }

        [PrimitiveValue(12)]
        public uint ColliderId { get; set; }

        [PrimitiveValue(28)]
        public uint PhysicsInfoId { get; set; }

        [PrimitiveValue(36)]
        public uint PhmoId { get; set; }

        [PrimitiveArray(40, 8)]
        public float[] Params { get; set; }
    }
}
