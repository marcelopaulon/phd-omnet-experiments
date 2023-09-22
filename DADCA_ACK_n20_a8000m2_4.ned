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

network DADCA_ACK_n20_a8000m2_4
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
        @display("i=misc/sensor;p=235.11,7833.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1162.29,7978.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1846.16,7859.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3862.74,7750.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5857.62,7764.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=833.69,7725.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1544.55,7591.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2137.53,7600.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2630.12,7523.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3277.44,7648.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5367.25,7535.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=214.61,7347.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1082.56,7455.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3804.64,7383.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4755.85,7475.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5080.45,7286.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=750.87,7148.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1413.92,7073.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1854.09,7141.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2777.89,7090.03");
    }

}

