import React from 'react';
import ReactDOM from 'react-dom/client';
import { RawMessagesPanel } from './components/RawMessages/RawMessagesPanel';
import './global.css';

class ErrorBoundary extends React.Component<{children: React.ReactNode}, {hasError: boolean; error: any}> {
  constructor(props: {children: React.ReactNode}) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: any) {
    return { hasError: true, error };
  }

  componentDidCatch(error: any, errorInfo: any) {
    console.error('Component Error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return (
        <div style={{color: 'white', padding: '20px'}}>
          <h1>Something went wrong.</h1>
          <pre style={{color: 'red'}}>{String(this.state.error)}</pre>
        </div>
      );
    }

    return this.props.children;
  }
}

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <ErrorBoundary>
      <RawMessagesPanel />
    </ErrorBoundary>
  </React.StrictMode>,
);
