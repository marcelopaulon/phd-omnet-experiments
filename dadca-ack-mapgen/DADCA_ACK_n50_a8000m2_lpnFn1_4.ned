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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor;p=116.21,179.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=39.99,156.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=189.97,105.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=165.01,226.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=252.09,233.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=205.97,298.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=157.83,378.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=127.48,445.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=199.22,494.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=204.62,427.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=104.47,544.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=34.29,560.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3.82,464.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=287.77,467.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=333.80,524.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=402.20,519.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=466.74,528.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=519.61,573.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=537.31,489.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=589.41,441.13");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=640.52,377.16");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=543.61,377.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=477.62,340.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=606.22,318.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=638.63,258.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=578.69,188.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=532.51,228.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=740.88,422.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=834.01,458.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=877.55,412.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=956.43,372.23");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1046.55,451.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=983.76,508.03");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=943.52,441.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=926.07,537.28");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=922.98,295.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=834.96,292.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=764.02,339.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=704.46,320.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=721.70,230.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=702.60,97.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=605.29,85.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=524.80,92.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=464.42,100.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=367.17,104.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=282.70,141.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=307.09,305.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=386.42,340.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=71.56,393.43");
    }

}

