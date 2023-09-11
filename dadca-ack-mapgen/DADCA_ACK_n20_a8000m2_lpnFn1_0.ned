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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=30.56,105.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=34.49,40.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=201.08,88.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=196.81,4.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=125.85,4.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=125.38,206.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=55.85,189.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=258.41,107.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=135.03,298.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=176.50,358.47");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=35.99,277.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=55.53,364.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=79.84,428.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=69.79,498.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=3.29,498.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=20.78,595.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=57.38,674.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=59.94,744.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=146.06,510.25");
    }

}

