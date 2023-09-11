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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor;p=139.20,158.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=61.07,171.45");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=183.35,61.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=30.98,101.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=46.95,32.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=262.82,61.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=322.94,39.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=400.12,61.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=395.56,1.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=537.80,51.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=484.54,21.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=458.78,159.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=265.12,138.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=306.10,193.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=357.06,270.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=383.16,175.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=356.26,110.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=590.61,165.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=682.80,173.28");
    }

}

