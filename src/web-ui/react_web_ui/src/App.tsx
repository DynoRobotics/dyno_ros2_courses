import { useState } from 'react'
import {
  ChakraProvider,
  Box,
  Heading,
  extendTheme,
  useColorMode,
  Button,
  HStack,
  Spacer,
  Container
} from '@chakra-ui/react'
import LayoutManager from './components/layout-manager'
import { WebSocketProvider } from './contexts/WebSocketContext'
import type { LayoutConfig } from './services/layoutService'

import './App.css'

// Create a custom theme that supports dark mode
const theme = extendTheme({
  config: {
    initialColorMode: 'light',
    useSystemColorMode: false,
  },
  styles: {
    global: (props: any) => ({
      body: {
        bg: props.colorMode === 'dark' ? 'gray.800' : 'white',
        color: props.colorMode === 'dark' ? 'white' : 'gray.800',
      },
    }),
  },
})

// Color mode toggle component
function ColorModeToggle() {
  const { colorMode, toggleColorMode } = useColorMode()

  return (
    <Button
      onClick={toggleColorMode}
      variant="ghost"
      size="sm"
      aria-label="Toggle color mode"
    >
      {colorMode === 'light' ? '🌙' : '☀️'}
    </Button>
  )
}

function App() {
  const [currentLayout, setCurrentLayout] = useState<LayoutConfig | null>(null)

  const handleLayoutChange = (layout: LayoutConfig) => {
    setCurrentLayout(layout)
    console.log('Layout changed:', layout.name)
  }

  return (
    <ChakraProvider theme={theme}>
      <WebSocketProvider>
        <Box minH="100vh">
          {/* Header with title and dark mode toggle */}
          <HStack p={4} bg="gray.50" _dark={{ bg: "gray.900" }}>
            <Heading as="h1" size="lg">
              ROS2 Web UI - Custom Layout Dashboard
            </Heading>
            <Spacer />
            <ColorModeToggle />
          </HStack>

          {/* Main content */}
          <LayoutManager onLayoutChange={handleLayoutChange} />
        </Box>
      </WebSocketProvider>
    </ChakraProvider >
  )
}

export default App
