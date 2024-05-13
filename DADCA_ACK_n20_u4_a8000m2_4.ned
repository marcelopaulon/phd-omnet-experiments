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
        @display("i=misc/sensor;p=344.75,7758.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1470.41,7948.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2113.34,7907.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2977.32,7853.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3404.56,7939.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3857.68,7896.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4887.45,7982.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5339.85,7801.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=829.70,7690.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1402.58,7554.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1962.24,7560.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2455.07,7622.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4881.40,7559.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5927.62,7577.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=188.14,7433.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1032.77,7368.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3410.89,7408.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4319.90,7459.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5360.24,7341.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=449.82,7192.41");
    }

}

