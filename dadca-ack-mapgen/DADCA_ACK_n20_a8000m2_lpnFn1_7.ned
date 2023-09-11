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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=180.66,75.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=130.02,2.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=219.88,124.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=262.38,80.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=238.99,18.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=284.47,145.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=297.06,235.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=381.78,283.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=397.65,345.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=382.79,198.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=348.08,141.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=442.36,178.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=436.17,92.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=229.94,306.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=152.04,302.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=153.56,400.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=221.34,440.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=285.52,394.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=330.80,353.26");
    }

}

