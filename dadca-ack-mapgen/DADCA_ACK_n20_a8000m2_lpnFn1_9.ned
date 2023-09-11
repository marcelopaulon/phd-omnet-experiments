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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=33.00,143.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=84.26,179.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=29.61,218.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=167.51,114.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=251.52,147.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=121.52,37.67");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=182.21,46.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=320.41,138.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=286.75,204.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=401.67,229.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=462.29,216.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=518.31,140.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=452.60,120.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=586.24,151.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=594.87,216.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=563.16,284.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=353.29,300.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=343.04,397.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=459.63,408.87");
    }

}

