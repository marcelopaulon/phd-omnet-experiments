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

network DADCA_ACK_n20_u16_a8000m2_7
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
        @display("i=misc/sensor;p=313.48,7911.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=673.49,7841.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1602.07,7993.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2304.73,7951.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3013.80,7930.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3911.36,7790.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4272.72,7899.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=140.10,7584.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=783.80,7508.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1241.07,7661.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1791.25,7691.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2726.03,7649.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2071.66,7373.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3189.88,7341.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3943.51,7312.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=328.99,7100.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1104.32,7074.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1555.78,7146.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2568.99,7060.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4409.37,7237.10");
    }

}

