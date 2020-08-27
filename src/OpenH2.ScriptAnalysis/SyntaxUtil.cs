﻿using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using OpenH2.Core.Extensions;
using OpenH2.Core.Scripting;
using OpenH2.Core.Tags.Scenario;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text.RegularExpressions;
using static Microsoft.CodeAnalysis.CSharp.SyntaxFactory;

namespace OpenH2.ScriptAnalysis
{
    public static class ScriptGenAnnotations
    {
        public static SyntaxAnnotation ResultStatement { get; } = new SyntaxAnnotation("ResultStatement");
        public static SyntaxAnnotation FinalScopeStatement { get; } = new SyntaxAnnotation("FinalScopeStatement");
        public static SyntaxAnnotation IfStatement { get; } = new SyntaxAnnotation("IfStatement");
        public static SyntaxAnnotation HoistedResultVar { get; } = new SyntaxAnnotation("HoistedResultVar");

        public const string TypeAnnotationKind = "TypeAnnotation";
        public static SyntaxAnnotation TypeAnnotation(ScriptDataType t) => new SyntaxAnnotation(TypeAnnotationKind, ((int)t).ToString());
    }

    public static class SyntaxUtil
    {
        public static TypeSyntax ScriptTypeSyntax(ScriptDataType dataType)
        {
            return dataType switch
            {
                ScriptDataType.Float => PredefinedType(Token(SyntaxKind.FloatKeyword)),
                ScriptDataType.Int => PredefinedType(Token(SyntaxKind.IntKeyword)),
                ScriptDataType.Boolean => PredefinedType(Token(SyntaxKind.BoolKeyword)),
                ScriptDataType.Short => PredefinedType(Token(SyntaxKind.IntKeyword)),
                ScriptDataType.String => PredefinedType(Token(SyntaxKind.StringKeyword)),
                ScriptDataType.Void => PredefinedType(Token(SyntaxKind.VoidKeyword)),
                
                _ => Enum.IsDefined(typeof(ScriptDataType), dataType) 
                    ? ParseTypeName(dataType.ToString())
                    : ParseTypeName(nameof(ScriptDataType) + dataType.ToString()),
            };
        }

        public static TypeSyntax TypeSyntax(Type dataType)
        {
            if(typeSyntaxCreators.TryGetValue(dataType, out var func))
            {
                return func();
            }

            return ParseTypeName(dataType.ToString());
        }

        private static Dictionary<Type, Func<TypeSyntax>> typeSyntaxCreators = new Dictionary<Type, Func<TypeSyntax>>()
        {
            { typeof(int), () => PredefinedType(Token(SyntaxKind.IntKeyword)) },
            { typeof(short),() => PredefinedType(Token(SyntaxKind.ShortKeyword)) },
            { typeof(float), () => PredefinedType(Token(SyntaxKind.FloatKeyword)) },
            { typeof(string), () => PredefinedType(Token(SyntaxKind.StringKeyword)) },
            { typeof(bool), () => PredefinedType(Token(SyntaxKind.BoolKeyword)) },
        };

        private static Dictionary<Type, ScriptDataType> toScriptTypeMap = new Dictionary<Type, ScriptDataType>()
        {
            { typeof(int), ScriptDataType.Int },
            { typeof(short), ScriptDataType.Short },
            { typeof(float), ScriptDataType.Float },
            { typeof(string), ScriptDataType.String },
            { typeof(bool), ScriptDataType.Boolean },
            { typeof(Team), ScriptDataType.Team },
            { typeof(AI), ScriptDataType.AI },
            { typeof(AIBehavior), ScriptDataType.AIBehavior },
            { typeof(DamageState), ScriptDataType.DamageState },
        };

        private static Dictionary<ScriptDataType, Type> toTypeMap = toScriptTypeMap.ToDictionary(kv => kv.Value, kv => kv.Key);

        public static ScriptDataType ScriptTypeFromType(Type t)
        {
            if(toScriptTypeMap.TryGetValue(t, out var val))
            {
                return val;
            }

            throw new Exception($"No mapping for '{t.Name}'");
        }

        public static bool TryGetTypeFromScriptType(ScriptDataType s, out Type t)
        {
            return toTypeMap.TryGetValue(s, out t);
        }

        private static Regex IdentifierInvalidChars = new Regex("[^\\dA-Za-z]", RegexOptions.Compiled);
        public static string SanitizeIdentifier(string name)
        {
            if(string.IsNullOrEmpty(name))
            {
                return name;
            }

            // BUGBUG: identifiers that only differ on separator characters would collide after this
            name = IdentifierInvalidChars.Replace(name, "_");

            if (char.IsDigit(name[0]))
            {
                name = "_" + name;
            }

            if(Array.IndexOf(_keywords, name) >= 0)
            {
                name = "@" + name;
            }

            return name;
        }

        public static string SanitizeMemberAccess(string name)
        {
            if (string.IsNullOrEmpty(name))
            {
                return name;
            }

            var segments = name.Split('.');

            for (var i = 0; i < segments.Length; i++)
            {
                segments[i] = SanitizeIdentifier(segments[i]);
            }

            return string.Join('.', segments);
        }

        public static FieldDeclarationSyntax CreateField(ScenarioTag.ScriptVariableDefinition variable)
        {
            return FieldDeclaration(VariableDeclaration(ScriptTypeSyntax(variable.DataType))
                    .WithVariables(SingletonSeparatedList<VariableDeclaratorSyntax>(
                        VariableDeclarator(SanitizeIdentifier(variable.Description)))))
                .WithAdditionalAnnotations(ScriptGenAnnotations.TypeAnnotation(variable.DataType));
        }

        public static PropertyDeclarationSyntax CreateProperty(ScriptDataType type, string name)
        {
            return PropertyDeclaration(ScriptTypeSyntax(type), name)
                    .WithModifiers(TokenList(Token(SyntaxKind.PublicKeyword)))
                    .WithAccessorList(AutoPropertyAccessorList())
                    .WithAdditionalAnnotations(ScriptGenAnnotations.TypeAnnotation(type));
        }

        public static IReadOnlyList<ScriptDataType> StringLiteralTypes = new ScriptDataType[]
        {
            ScriptDataType.String,
            ScriptDataType.StringId,
            ScriptDataType.ReferenceGet,
            ScriptDataType.Animation,
            ScriptDataType.Weapon,
            ScriptDataType.SpatialPoint,
            ScriptDataType.WeaponReference,
            ScriptDataType.GameDifficulty,
            ScriptDataType.VehicleSeat
        };

        public static IReadOnlyList<ScriptDataType> NumericLiteralTypes = new ScriptDataType[]
        {
            ScriptDataType.Float,
            ScriptDataType.Int,
            ScriptDataType.Short
        };

        public static LiteralExpressionSyntax LiteralExpression(ScenarioTag tag, ScenarioTag.ScriptSyntaxNode node, ScriptDataType destinationType)
        {
            if (StringLiteralTypes.Contains(node.DataType))
            {
                return LiteralExpression(GetScriptString(tag, node))
                    .WithAdditionalAnnotations(ScriptGenAnnotations.TypeAnnotation(destinationType));
            }

            object nodeValue = node.DataType switch
            {
                ScriptDataType.Float => BitConverter.Int32BitsToSingle((int)node.NodeData_32),
                ScriptDataType.Int => (int)node.NodeData_32,
                ScriptDataType.Boolean => node.NodeData_B3 == 1,
                ScriptDataType.Short => (short)node.NodeData_H16,

                _ => throw new NotImplementedException(),
            };


            var exp = destinationType switch
            {
                ScriptDataType.Float => LiteralExpression(Convert.ToSingle(nodeValue)),
                ScriptDataType.Int => LiteralExpression(Convert.ToInt32(nodeValue)),
                ScriptDataType.Boolean => LiteralExpression(Convert.ToBoolean(nodeValue)),
                ScriptDataType.Short => LiteralExpression(Convert.ToInt16(nodeValue)),

                _ => throw new NotImplementedException(),
            };

            return exp.WithAdditionalAnnotations(ScriptGenAnnotations.TypeAnnotation(destinationType));
        }

        public static string GetScriptString(ScenarioTag tag, ScenarioTag.ScriptSyntaxNode node)
        {
            return ((Span<byte>)tag.ScriptStrings).ReadStringStarting(node.NodeString);
        }

        public static LiteralExpressionSyntax LiteralExpression<T>(T value)
        {
            return value switch
            {
                int i => SyntaxFactory.LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(i)),
                short s => SyntaxFactory.LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(s)),
                ushort s => SyntaxFactory.LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(s)),
                float f => SyntaxFactory.LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(f)),
                string s => SyntaxFactory.LiteralExpression(SyntaxKind.StringLiteralExpression, Literal(s)),
                true => SyntaxFactory.LiteralExpression(SyntaxKind.TrueLiteralExpression),
                false => SyntaxFactory.LiteralExpression(SyntaxKind.FalseLiteralExpression),

                _ => throw new NotImplementedException(),
            };
        }

        public static bool TryGetContainingSimpleExpression(this StatementSyntax statement, out ExpressionSyntax simple)
        {
            if (statement is ReturnStatementSyntax ret)
            {
                simple = ret.Expression.IsSimpleExpression() ? ret.Expression : default;
            }
            else if (statement is ExpressionStatementSyntax exp)
            {
                simple = exp.Expression.IsSimpleExpression() ? exp.Expression : default;
            }
            else
            {
                simple = default;
            }

            return simple != default;
        }

        private static Type[] simpleExpressionTypes = new Type[]
        {
            typeof(CastExpressionSyntax),
            typeof(IdentifierNameSyntax),
            typeof(LiteralExpressionSyntax),
            typeof(InvocationExpressionSyntax),
            typeof(MemberAccessExpressionSyntax),
            typeof(ParenthesizedExpressionSyntax),
            typeof(BinaryExpressionSyntax),
            typeof(PrefixUnaryExpressionSyntax),
            typeof(PostfixUnaryExpressionSyntax),
        };

        public static bool IsSimpleExpression(this ExpressionSyntax exp)
        {
            return Array.IndexOf(simpleExpressionTypes, exp.GetType()) >= 0;
        }

        public static bool TryGetLeftHandExpression(this StatementSyntax statement, out ExpressionSyntax rhs)
        {
            rhs = statement switch
            {
                ExpressionStatementSyntax exp => HandleExpression(exp.Expression),
                ReturnStatementSyntax ret => HandleExpression(ret.Expression),

                _ => default
            };

            return rhs != default;

            ExpressionSyntax HandleExpression(ExpressionSyntax exp)
            {
                return exp switch
                {
                    AssignmentExpressionSyntax ass => ass.Left,

                    _ => default
                };
            }
        }

        public static ExpressionSyntax CreateCast(Type from, Type to, ExpressionSyntax inner)
        {
            if(to == typeof(bool) && NumericTypes.Contains(from))
            {
                // Generate comparison conditional result
                return ConditionalExpression(
                    BinaryExpression(SyntaxKind.EqualsExpression,
                        inner,
                        Token(SyntaxKind.EqualsEqualsToken),
                        LiteralExpression(1)),
                    LiteralExpression(true),
                    LiteralExpression(false));
            }
            else if (NumericTypes.Contains(to) && from == typeof(bool))
            {
                return ConditionalExpression(inner, LiteralExpression(1), LiteralExpression(0));
            }
            else
            {
                return CastExpression(TypeSyntax(to), inner);
            }
        }

        public static ExpressionSyntax CreateCast(ScriptDataType from, ScriptDataType to, ExpressionSyntax inner)
        {
            if(toTypeMap.TryGetValue(from, out var fromT) &&
                toTypeMap.TryGetValue(to, out var toT))
            {
                return CreateCast(fromT, toT, inner);
            }
            else
            {
                return inner.WithTrailingTrivia(Comment($"// Couldn't generate cast from '{from}' to '{to}'"));
            }
        }

        private static readonly HashSet<Type> NumericTypes = new HashSet<Type>
        {
            typeof(int),  typeof(double),  typeof(decimal),
            typeof(long), typeof(short),   typeof(sbyte),
            typeof(byte), typeof(ulong),   typeof(ushort),
            typeof(uint), typeof(float)
        };

        public static ScriptDataType BinaryNumericPromotion(ScriptDataType a, ScriptDataType b)
        {
            Debug.Assert(NumericLiteralTypes.Contains(a) && NumericLiteralTypes.Contains(b));

            if (a == ScriptDataType.Float || b == ScriptDataType.Float)
                return ScriptDataType.Float;
            else
                return ScriptDataType.Int;
        }

        public static bool TryGetTypeOfExpression(ExpressionSyntax exp, out ScriptDataType type)
        {
            var ext = new TypeExtractor();
            ext.Visit(exp);

            type = ext.Type ?? default;
            return ext.Type != default;
        }

        private class TypeExtractor : CSharpSyntaxWalker
        {
            public ScriptDataType? Type { get; private set; } = null;


            public override void Visit(SyntaxNode node)
            {
                if(node.HasAnnotations(ScriptGenAnnotations.TypeAnnotationKind))
                {
                    var types = node.GetAnnotations(ScriptGenAnnotations.TypeAnnotationKind);

                    this.Type = (ScriptDataType)int.Parse(types.First().Data);
                    return;
                }

                base.Visit(node);
            }
        }

        public static ExpressionSyntax CreateImmediatelyInvokedFunction(ScriptDataType returnType, IEnumerable<StatementSyntax> body)
        {
            ObjectCreationExpressionSyntax funcObj;

            if (returnType == ScriptDataType.Void)
            {
                funcObj = SyntaxFactory.ObjectCreationExpression(
                    SyntaxFactory.IdentifierName("Action"));
            }
            else
            {
                funcObj = SyntaxFactory.ObjectCreationExpression(
                    SyntaxFactory.GenericName("Func")
                        .AddTypeArgumentListArguments(SyntaxUtil.ScriptTypeSyntax(returnType)));
            }

            return InvocationExpression(
                funcObj.AddArgumentListArguments(
                    Argument(
                        ParenthesizedLambdaExpression(
                            Block(
                                List(body))))))
                .WithAdditionalAnnotations(ScriptGenAnnotations.TypeAnnotation(returnType));
        }

        public static AccessorListSyntax AutoPropertyAccessorList()
        {
            return AccessorList(List<AccessorDeclarationSyntax>(new[] {
                AccessorDeclaration(SyntaxKind.GetAccessorDeclaration)
                    .WithSemicolonToken(Token(TriviaList(), SyntaxKind.SemicolonToken, TriviaList(Space))),
                AccessorDeclaration(SyntaxKind.SetAccessorDeclaration)
                    .WithSemicolonToken(Token(TriviaList(), SyntaxKind.SemicolonToken, TriviaList(Space)))
            }));
        }

        public static CSharpSyntaxNode Normalize(CSharpSyntaxNode node)
        {
            node = node.NormalizeWhitespace();

            var norm = new DeNormalizer();
            var newNode = norm.Visit(node);


            return newNode as CSharpSyntaxNode;
        }

        public class DeNormalizer : CSharpSyntaxRewriter
        {
            private AccessorListSyntax autoPropList = AccessorList(List<AccessorDeclarationSyntax>(new[] {
                AccessorDeclaration(SyntaxKind.GetAccessorDeclaration)
                    .WithSemicolonToken(Token(TriviaList(), SyntaxKind.SemicolonToken, TriviaList(Space))),
                AccessorDeclaration(SyntaxKind.SetAccessorDeclaration)
                    .WithSemicolonToken(Token(TriviaList(), SyntaxKind.SemicolonToken, TriviaList(Space)))
            }));

            public override SyntaxNode? VisitPropertyDeclaration(PropertyDeclarationSyntax node)
            {
                if (AreEquivalent(node.AccessorList, autoPropList, false))
                {
                    var newNode = node
                        .WithIdentifier(node.Identifier.WithTrailingTrivia(Space))
                        .WithAccessorList(autoPropList.WithOpenBraceToken(
                            Token(
                                TriviaList(),
                                SyntaxKind.OpenBraceToken,
                                TriviaList(
                                    Space)))
                            .WithCloseBraceToken(
                                Token(
                                    TriviaList(),
                                    SyntaxKind.CloseBraceToken,
                                    TriviaList(
                                        LineFeed))));


                    return base.VisitPropertyDeclaration(newNode);
                }

                return base.VisitPropertyDeclaration(node);
            }

            public override SyntaxNode VisitArgumentList(ArgumentListSyntax node)
            {
                var newArgs = node.Arguments;

                if(newArgs.Count > 1)
                {
                    for (var i = 0; i < node.Arguments.Count - 1; i++)
                    {
                        newArgs = newArgs.ReplaceSeparator(newArgs.GetSeparator(i), Token(SyntaxKind.CommaToken).WithTrailingTrivia(Space));
                    }
                }

                node = node.WithCloseParenToken(node.CloseParenToken.WithLeadingTrivia())
                    .WithArguments(newArgs);

                return base.VisitArgumentList(node);
            }

            public override SyntaxNode VisitParenthesizedLambdaExpression(ParenthesizedLambdaExpressionSyntax node)
            {
                var newNode = node;

                if(newNode.Body is BlockSyntax block)
                {
                    newNode = newNode.WithBody(block.WithCloseBraceToken(block.CloseBraceToken.WithTrailingTrivia()));
                }

                return base.VisitParenthesizedLambdaExpression(newNode);
            }
        }

        private static readonly string[] _keywords = new[]
        {
            "abstract",  "event",      "new",        "struct",
            "as",        "explicit",   "null",       "switch",
            "base",      "extern",     "object",     "this",
            "bool",      "false",      "operator",   "throw",
            "break",     "finally",    "out",        "true",
            "byte",      "fixed",      "override",   "try",
            "case",      "float",      "params",     "typeof",
            "catch",     "for",        "private",    "uint",
            "char",      "foreach",    "protected",  "ulong",
            "checked",   "goto",       "public",     "unchekeced",
            "class",     "if",         "readonly",   "unsafe",
            "const",     "implicit",   "ref",        "ushort",
            "continue",  "in",         "return",     "using",
            "decimal",   "int",        "sbyte",      "virtual",
            "default",   "interface",  "sealed",     "volatile",
            "delegate",  "internal",   "short",      "void",
            "do",        "is",         "sizeof",     "while",
            "double",    "lock",       "stackalloc",
            "else",      "long",       "static",
            "enum",      "namespace",  "string",
            // contextual keywords 
            "add",       "alias",       "ascending",  "async",
            "await",      "by",         "descending", "dynamic",
            "equals",    "from",       "get",        "global",
            "group",      "into",       "join",       "let",
            "nameof",    "on",         "orderby",    "partial",
            "remove",    "select",     "set",        "value",
            "var",       "when",       "where",      "yield"
        };
    }
}
