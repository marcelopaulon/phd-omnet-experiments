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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=6.69,108.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=36.46,182.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=117.50,200.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=253.27,185.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=199.13,100.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=173.80,39.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=24.44,27.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=276.50,57.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=325.15,119.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=346.36,208.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=400.34,159.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=446.03,103.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=378.66,45.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=508.46,1.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=549.96,82.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=544.47,158.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=619.54,123.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=679.45,129.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=618.63,185.03");
    }

}

