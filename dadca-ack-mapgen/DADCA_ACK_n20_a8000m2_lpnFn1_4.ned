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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=162.34,107.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=233.54,89.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=169.24,23.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=65.89,40.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=134.85,237.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=47.16,218.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=200.43,177.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=273.79,212.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=285.01,310.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=351.01,233.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=255.37,381.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=306.70,445.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=370.35,428.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=361.33,318.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=418.47,367.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=361.05,518.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=192.11,456.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=172.85,355.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=423.67,524.95");
    }

}

