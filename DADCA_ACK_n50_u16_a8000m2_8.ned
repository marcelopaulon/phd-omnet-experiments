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

network DADCA_ACK_n50_u16_a8000m2_8
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
        @display("i=misc/sensor;p=296.35,7787.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=723.07,7800.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1385.41,7861.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2280.69,7945.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2767.47,7949.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3146.81,7868.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3792.57,7885.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4538.86,7856.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4962.34,7935.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5635.64,7995.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1736.03,7704.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4273.59,7615.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5416.31,7697.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5857.69,7674.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=48.68,7344.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=773.81,7421.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1208.93,7469.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1743.11,7277.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2138.79,7496.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2971.03,7502.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3399.85,7420.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3841.93,7450.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4653.33,7429.36");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=304.78,7044.70");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1039.64,7013.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2078.97,7112.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2511.65,7225.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4269.56,7193.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5308.32,7212.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2191.29,6764.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2624.07,6769.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2975.77,6821.33");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3461.22,6794.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4392.95,6840.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4739.79,6923.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=30.39,6720.65");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=546.82,6647.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1325.85,6667.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1738.03,6759.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4003.09,6720.81");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5232.90,6580.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=364.65,6285.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=769.62,6316.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1392.97,6306.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2024.93,6426.42");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2892.52,6416.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4499.80,6321.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4903.82,6377.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1756.54,6059.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2627.73,6092.88");
    }

}

