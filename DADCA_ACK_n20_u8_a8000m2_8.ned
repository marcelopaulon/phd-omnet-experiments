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

network DADCA_ACK_n20_u8_a8000m2_8
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
        @display("i=misc/sensor;p=511.25,7857.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1213.94,7991.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1811.87,7770.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2715.50,7898.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3349.25,7846.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4119.09,7885.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5235.08,7850.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=9.87,7706.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=879.12,7688.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1401.92,7605.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3633.55,7607.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4550.86,7610.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5641.03,7720.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=551.90,7288.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=995.61,7282.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2002.80,7310.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2528.16,7353.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2887.33,7349.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3277.86,7496.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4136.83,7414.28");
    }

}

