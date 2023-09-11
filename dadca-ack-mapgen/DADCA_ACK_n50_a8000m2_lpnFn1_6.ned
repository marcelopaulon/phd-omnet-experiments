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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=33.90,171.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=123.76,196.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=173.88,258.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=222.59,310.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=245.62,402.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=298.07,442.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=356.46,279.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=326.19,189.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=401.34,226.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=343.82,367.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=356.06,468.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=364.52,539.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=414.44,603.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=465.49,547.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=422.87,460.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=237.72,483.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=402.41,399.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=497.09,492.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=490.63,620.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=556.52,387.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=557.02,314.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=598.20,229.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=652.40,337.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=701.81,253.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=682.20,185.64");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=771.20,270.23");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=825.81,198.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=910.48,221.43");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=972.81,272.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=1009.22,198.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=743.34,142.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=869.54,269.32");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=886.82,340.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=825.47,82.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=751.79,361.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=784.48,449.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=753.13,542.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=843.95,543.77");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=790.17,591.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=661.90,591.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=686.67,532.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=888.07,457.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=901.66,663.57");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=840.94,630.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=783.66,660.24");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=910.29,732.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=930.85,803.90");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=906.69,870.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=854.90,821.13");
    }

}

