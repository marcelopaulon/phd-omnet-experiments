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
        @display("i=misc/sensor,p=113.23,163.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=91.83,0.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=173.08,54.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=227.80,131.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=264.46,83.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=277.06,23.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=367.33,166.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=335.12,249.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=418.64,300.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=394.16,221.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=505.81,254.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=570.27,271.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=569.18,211.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=509.47,174.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=553.93,112.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=646.46,78.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=719.46,84.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=724.11,3.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=801.33,58.62");
    }

}

