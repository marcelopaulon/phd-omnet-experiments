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

network DADCA_ACK_n50_u3_a8000m2_5
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
        @display("i=misc/sensor;p=1174.11,7775.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1794.11,7859.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2520.77,7988.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3136.97,7883.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3749.06,7938.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4251.78,7761.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4894.19,7762.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5283.79,7824.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=731.85,7662.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2146.46,7672.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5954.05,7565.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=182.27,7491.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=837.93,7297.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1170.77,7419.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2671.88,7348.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3175.15,7368.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3572.13,7383.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4051.88,7358.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4496.85,7423.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5602.15,7447.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=475.55,7178.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1166.69,7065.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1620.60,7241.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3959.46,7015.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4970.47,7166.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5453.11,7100.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=155.00,6991.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1992.54,6794.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2346.60,6999.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2716.58,6839.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3105.75,6850.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3552.21,7009.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4561.52,6983.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5887.97,6960.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=292.59,6543.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=860.24,6692.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1219.00,6583.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3695.67,6609.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4963.42,6554.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=839.52,6310.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2473.72,6327.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4176.22,6380.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5331.33,6475.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5878.29,6466.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1293.73,6195.78");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1695.21,6226.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2971.02,6263.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4844.70,6184.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5567.14,6044.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=319.86,5970.99");
    }

}

