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

network DADCA_ACK_n20_u4_a8000m2_4
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
        @display("i=misc/sensor;p=1336.24,7757.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3386.94,7960.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4388.90,7930.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4975.01,7793.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5590.07,7838.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=192.91,7693.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=894.67,7668.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2152.18,7718.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2573.68,7661.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3202.01,7520.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3968.93,7728.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5232.89,7545.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=476.44,7305.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1899.37,7408.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3658.25,7420.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5952.22,7267.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=118.53,7081.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1162.90,7197.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1565.13,7228.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2297.91,7134.67");
    }

}

