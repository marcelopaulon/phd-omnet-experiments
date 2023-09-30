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

network DADCA_ACK_n50_u4_a8000m2_9
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
        @display("i=misc/sensor;p=1342.59,7875.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1853.89,7974.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2209.99,7777.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2877.42,7811.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3691.24,7860.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4077.34,7890.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5267.79,7929.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=998.00,7642.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1606.37,7591.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3721.04,7505.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4661.02,7735.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5920.08,7699.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=437.46,7464.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1236.72,7263.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2108.38,7399.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2761.59,7348.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4193.10,7323.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4846.97,7377.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5403.96,7415.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1823.84,7024.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2274.25,7026.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3393.87,7125.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5690.25,7189.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=644.94,6972.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1055.55,6852.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2616.65,6794.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3834.79,6994.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4342.28,6983.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4907.04,6990.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1381.01,6555.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2317.41,6610.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3071.44,6741.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4792.83,6517.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5193.21,6641.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5755.00,6713.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=471.20,6432.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1896.51,6512.01");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2209.10,6277.24");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2904.98,6335.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3411.69,6311.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4021.99,6475.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=790.15,6076.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1433.63,6096.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2562.76,6149.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5858.66,6158.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2.54,5928.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1844.50,6015.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3481.14,5883.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3958.02,5969.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4383.74,5901.05");
    }

}

