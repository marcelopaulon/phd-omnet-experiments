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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=167.33,138.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=7.89,139.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=33.63,226.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=124.36,205.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=180.06,232.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=250.46,214.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=262.09,315.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=320.88,331.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=358.70,379.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=306.05,440.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=238.01,385.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=351.91,216.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=387.75,306.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=480.57,445.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=491.55,539.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=419.27,599.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=471.28,678.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=337.19,617.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=282.22,592.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=328.56,521.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=391.16,505.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=419.05,435.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=182.53,563.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=177.79,485.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=214.91,656.20");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=294.11,711.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=426.39,728.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=393.09,671.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=307.76,773.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=235.44,841.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=332.31,863.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=387.93,812.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=424.83,869.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=297.75,936.95");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=399.35,932.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=239.31,737.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=173.82,747.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=89.01,757.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=104.89,816.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=10.73,848.45");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=41.93,908.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=526.07,401.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=535.44,589.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=548.53,670.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=619.02,619.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=622.57,533.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=643.98,472.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=269.28,491.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=490.78,775.29");
    }

}

