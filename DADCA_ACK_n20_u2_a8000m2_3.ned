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

network DADCA_ACK_n20_u2_a8000m2_3
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
        @display("i=misc/sensor;p=140.22,7777.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=823.51,7819.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1816.08,7770.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2268.91,7961.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2613.19,7764.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4313.27,7868.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4978.25,7985.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5519.30,7881.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=599.71,7545.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1424.42,7730.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4657.52,7748.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5341.99,7517.03");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=308.65,7298.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1083.80,7271.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1732.59,7267.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2122.02,7346.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2797.02,7323.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5024.02,7333.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=751.78,7142.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4371.85,7214.33");
    }

}

