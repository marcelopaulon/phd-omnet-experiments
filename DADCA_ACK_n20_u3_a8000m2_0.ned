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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=590.71,7871.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1628.36,7813.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2765.69,7937.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3374.25,7799.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3868.81,7765.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4536.31,7969.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=355.22,7535.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1253.63,7735.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4271.98,7678.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5234.16,7576.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5649.38,7668.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=863.20,7292.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1321.23,7264.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1721.84,7375.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2250.51,7308.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2918.40,7400.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3937.81,7370.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4502.95,7397.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4866.22,7369.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5861.08,7273.25");
    }

}

