package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_a8000m2_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=58.40,7998.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=135.08,7961.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=184.79,7997.33");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=249.39,7987.14");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=344.59,7971.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=645.79,7979.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=764.53,7992.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=835.31,7992.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=944.83,7992.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=998.25,7963.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=14.77,7941.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=444.26,7944.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=548.80,7922.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=707.25,7940.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=815.59,7918.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=67.91,7882.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=204.24,7907.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=285.48,7911.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=398.42,7902.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=981.29,7874.34");
    }

}

