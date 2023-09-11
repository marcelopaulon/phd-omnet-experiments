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
        @display("i=misc/sensor,p=179.67,54.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=247.14,111.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=316.17,109.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=368.71,73.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=425.20,32.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=485.98,76.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=538.70,12.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=632.21,51.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=632.98,118.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=562.48,129.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=629.47,180.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=735.81,70.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=773.31,162.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=862.53,123.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=827.97,203.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=833.66,276.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=740.51,276.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=930.34,116.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=910.84,184.17");
    }

}

