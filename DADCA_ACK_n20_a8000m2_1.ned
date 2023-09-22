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
        @display("i=misc/sensor;p=191.83,7934.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1483.49,7871.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2566.85,7815.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5363.25,7845.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=968.64,7589.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3730.30,7590.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4239.32,7527.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4577.43,7740.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5015.60,7656.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=373.88,7366.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1753.51,7433.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3016.07,7428.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4819.11,7358.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5507.44,7399.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1352.82,7088.26");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2084.33,7206.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2518.60,7240.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3359.20,7245.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3954.46,7221.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4470.89,7174.25");
    }

}

