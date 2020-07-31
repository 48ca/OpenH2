﻿using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using OpenH2.Core.Tags.Scenario;
using System;
using System.Collections.Generic;

namespace OpenH2.ScriptAnalysis.GenerationState
{
    public class StatementBlockContext : BaseGenerationContext, IGenerationContext, IStatementContext
    {
        public List<StatementSyntax> Statements { get; set; } = new List<StatementSyntax>();

        public StatementBlockContext() : base(null)
        {
        }

        public IGenerationContext AddExpression(ExpressionSyntax expression)
        {
            this.AddStatement(SyntaxFactory.ExpressionStatement(expression));
            return this;
        }

        IStatementContext IStatementContext.AddStatement(StatementSyntax statement) => AddStatement(statement);
        public StatementBlockContext AddStatement(StatementSyntax statement)
        {
            this.Statements.Add(statement);
            return this;
        }

        public StatementSyntax CreateResultStatement(ExpressionSyntax resultValue)
        {
            return SyntaxFactory.ReturnStatement(resultValue);
        }

        public StatementSyntax[] GetInnerStatements()
        {
            return Statements.ToArray();
        }

        public void GenerateInto(Scope scope)
        {
            throw new NotSupportedException("Statement block is a top-level scope and should not generate into another scope");
        }
    }
}
