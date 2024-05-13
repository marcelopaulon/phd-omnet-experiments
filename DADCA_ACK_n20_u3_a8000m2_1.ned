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

network DADCA_ACK_n20_u3_a8000m2_1
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
        @display("i=misc/sensor;p=136.96,7991.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=542.43,7894.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1069.76,7860.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2041.32,7846.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2561.55,7855.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3464.80,7866.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5000.73,7934.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5400.99,7942.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=228.97,7523.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1651.01,7664.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3051.12,7687.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3993.68,7587.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4939.47,7580.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5582.73,7625.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=806.29,7469.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1212.43,7271.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2230.03,7489.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3104.72,7305.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3469.71,7382.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4399.89,7420.59");
    }

}

