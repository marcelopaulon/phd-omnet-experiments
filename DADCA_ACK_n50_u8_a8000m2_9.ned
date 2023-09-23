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

network DADCA_ACK_n50_u8_a8000m2_9
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
        @display("i=misc/sensor;p=547.67,7904.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1058.33,7993.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1462.05,7905.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2493.61,7773.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3532.39,7939.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4008.88,7808.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4363.86,7774.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5077.35,7955.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5671.65,7852.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=434.39,7506.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2119.94,7712.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4770.07,7515.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5337.59,7528.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1016.34,7471.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1781.64,7482.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2664.05,7421.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3489.67,7443.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4164.19,7268.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1115.83,7100.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2170.74,7069.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2685.34,7057.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3328.50,7099.72");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3734.49,7108.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4752.74,7103.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5723.71,7073.26");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=297.52,6976.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1623.38,7008.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2902.85,6766.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4400.03,6927.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5069.81,6950.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=553.56,6682.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1017.97,6658.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1862.93,6594.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2221.83,6683.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3890.78,6727.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5110.55,6554.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5497.51,6635.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3.45,6360.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1372.92,6283.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3094.88,6467.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3516.11,6435.79");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4414.22,6298.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5698.51,6272.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=751.73,6117.19");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1689.65,6042.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2265.83,6186.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3894.52,6022.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2.24,5837.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2658.49,5988.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3253.18,5892.95");
    }

}

