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

network DADCA_ACK_n50_u8_a8000m2_8
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
        @display("i=misc/sensor;p=844.26,7997.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1203.97,7982.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2052.53,7978.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2449.96,7815.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3346.71,7768.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3919.17,7856.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4278.34,7781.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4971.43,7905.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1696.78,7560.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2091.58,7570.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2774.38,7559.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4785.43,7525.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5276.01,7509.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5640.79,7661.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=384.88,7409.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1199.21,7446.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3702.04,7301.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5943.75,7466.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1361.89,7122.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1998.58,7029.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2368.45,7189.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3199.83,7146.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4064.02,7197.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5348.59,7013.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=219.71,6773.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=821.06,6992.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1664.97,6870.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2781.01,6973.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3750.96,6860.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4380.80,6999.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5718.86,6981.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=584.55,6733.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1189.71,6541.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2350.29,6525.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3456.41,6545.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4146.73,6549.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=371.02,6333.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1563.36,6469.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1908.58,6345.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2635.95,6305.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3815.97,6346.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5209.37,6471.90");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5856.57,6300.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=767.57,6207.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3148.98,6267.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4373.12,6183.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4965.16,6168.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5496.73,6261.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1.33,5989.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1403.27,5963.15");
    }

}

