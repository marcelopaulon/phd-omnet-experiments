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
        @display("i=misc/sensor;p=81.97,7955.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1533.52,7989.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2467.36,7932.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2874.18,7984.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3545.31,7869.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4554.01,7879.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5317.59,7771.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5773.15,7835.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=720.13,7720.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1112.79,7692.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1829.58,7671.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3050.37,7597.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3700.73,7522.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4278.88,7559.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4917.35,7584.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=36.58,7347.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=740.86,7362.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1508.56,7477.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2513.19,7478.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3276.43,7304.66");
    }

}

