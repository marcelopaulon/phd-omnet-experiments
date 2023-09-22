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

network DADCA_ACK_n50_a8000m2_2
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
        @display("i=misc/sensor;p=253.70,7997.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=373.06,7998.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=652.02,7973.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=909.69,7981.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=166.87,7947.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=352.43,7914.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=433.55,7955.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=737.48,7953.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=813.24,7914.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=201.00,7877.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=277.02,7895.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=419.27,7891.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=500.63,7902.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=709.44,7879.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=872.52,7873.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=61.77,7869.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=525.55,7836.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=609.02,7848.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=6.05,7796.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=147.35,7808.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=323.39,7815.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=467.24,7807.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=797.08,7798.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=225.02,7780.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=412.94,7769.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=487.73,7741.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=596.91,7760.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=678.88,7745.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=867.13,7773.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=931.81,7780.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=19.30,7712.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=117.44,7740.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=193.25,7711.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=293.23,7719.91");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=409.40,7703.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=790.76,7711.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=140.17,7675.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=486.01,7672.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=594.86,7688.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=661.33,7657.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=908.37,7686.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=109.22,7617.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=213.41,7626.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=327.63,7630.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=554.76,7638.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=762.68,7640.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=959.71,7650.30");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=251.72,7576.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=399.57,7586.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=477.81,7595.37");
    }

}

