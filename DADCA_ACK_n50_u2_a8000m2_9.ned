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

network DADCA_ACK_n50_u2_a8000m2_9
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
        @display("i=misc/sensor;p=148.99,7834.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=837.53,7786.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1934.00,7956.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2433.86,7882.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3321.78,7895.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3762.58,7996.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4574.76,7963.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5388.84,7811.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1478.27,7673.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2126.73,7644.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3711.70,7644.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4170.03,7664.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4793.80,7669.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5966.94,7708.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=409.30,7455.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=958.41,7445.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1740.87,7397.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2779.50,7503.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3416.67,7272.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4536.49,7310.14");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5171.30,7491.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1513.25,7118.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1930.81,7055.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2381.01,7126.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4040.99,7079.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5111.00,7144.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5502.15,7198.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5857.97,7199.96");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=555.02,6887.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1073.87,6887.15");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2970.26,6907.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4443.77,6956.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=303.61,6566.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1491.40,6705.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2539.50,6618.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3271.64,6561.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3689.40,6712.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4083.05,6606.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4823.45,6585.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5379.56,6590.39");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5851.67,6552.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1212.66,6478.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1689.23,6301.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2024.66,6479.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2809.73,6392.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3837.66,6318.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4419.60,6468.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=70.24,6058.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=748.39,6103.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1437.25,6056.10");
    }

}

