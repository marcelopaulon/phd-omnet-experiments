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

network DADCA_ACK_n20_u2_a8000m2_2
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
        @display("i=misc/sensor;p=1105.77,7827.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1705.22,7766.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2999.57,7760.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4920.61,7953.87");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5587.10,7800.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5969.12,7949.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=95.86,7643.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=644.62,7685.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2602.78,7615.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3625.62,7627.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4276.63,7584.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=866.64,7273.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1384.67,7274.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1991.62,7312.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5016.65,7404.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5959.27,7423.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=352.51,7040.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2388.90,7251.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3026.37,7064.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3700.45,7182.80");
    }

}

