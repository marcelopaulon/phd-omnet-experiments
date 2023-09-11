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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=34.38,148.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=7.21,56.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=188.83,103.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=272.73,117.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=362.61,74.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=278.62,38.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=224.08,200.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=154.01,264.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=214.50,26.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=354.78,255.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=278.74,273.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=201.45,306.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=218.36,375.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=135.56,359.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=84.39,397.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=75.89,473.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=149.54,535.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=154.48,599.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=98.04,646.88");
    }

}

