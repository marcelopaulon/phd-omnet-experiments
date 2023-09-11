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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=183.16,150.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=15.04,124.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=52.10,39.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=165.34,89.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=127.03,29.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=235.03,36.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=319.36,25.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=301.60,89.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=122.86,173.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=179.52,217.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=151.15,288.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=116.48,238.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=65.43,386.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=51.57,455.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=57.25,531.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=153.43,527.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=215.74,566.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=166.21,632.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=143.14,721.68");
    }

}

