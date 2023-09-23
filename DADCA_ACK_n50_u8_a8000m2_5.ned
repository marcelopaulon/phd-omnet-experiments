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

network DADCA_ACK_n50_u8_a8000m2_5
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
        @display("i=misc/sensor;p=356.93,7834.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1436.20,7939.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2007.61,7785.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4714.79,7846.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5686.83,7903.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=990.94,7732.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2385.45,7729.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3175.60,7678.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4046.90,7639.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5093.44,7687.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=649.91,7267.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1220.12,7466.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1717.40,7448.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2654.53,7390.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3494.49,7433.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4619.69,7305.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5531.79,7293.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5866.92,7435.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1141.63,7073.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2316.19,7211.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2949.15,7032.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3424.67,7090.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4117.11,7243.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1570.02,6779.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2023.33,6826.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2478.21,6861.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3622.81,6770.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5301.92,6963.48");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5734.28,6939.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=543.87,6753.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1080.11,6632.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3027.29,6594.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4026.67,6748.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4648.39,6612.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5882.14,6612.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=307.77,6274.28");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2087.57,6325.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2435.55,6447.79");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4370.72,6347.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5002.36,6379.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=805.63,6191.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1310.85,6026.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1672.18,6214.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3008.97,6206.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3646.87,6167.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4682.55,6084.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5666.69,6264.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=32.10,5883.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2503.23,5796.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=456.38,5529.85");
    }

}

