﻿using System;
using System.Collections.Generic;
using System.Text;

namespace OpenH2.Core.Tags.Layout
{
    public abstract class TagValueAttribute : Attribute
    {
        public TagValueAttribute(int offset)
        {
            Offset = offset;
        }

        public int Offset { get; }
    }
}
